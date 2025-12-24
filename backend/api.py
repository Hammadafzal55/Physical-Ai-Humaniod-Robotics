import os
import logging
from typing import List, Dict, Any, Optional
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse # Added StreamingResponse import
import asyncio # Added for generator
import json # Added for potential error messages

from dotenv import load_dotenv

from retrieval import retrieve_context, RetrievedChunk, RetrievalServiceMetadata
from agent import GeminiRagAgent

# Load environment variables
load_dotenv()

# --- Configuration & Logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- FastAPI Application Instance ---
app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="API for retrieving information and chatting about the Physical AI and Humanoid Robotics textbook.",
    version="1.0.0"
)

# Added CORS middleware configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods (GET, POST, etc.)
    allow_headers=["*"],  # Allows all headers
)

# --- Agent Initialization ---
# Create a single, reusable instance of the GeminiRagAgent
# This is placed here to ensure the agent is initialized only once when the app starts.
try:
    rag_agent = GeminiRagAgent()
    logger.info("Gemini RAG Agent initialized successfully.")
except ValueError as e:
    logger.error(f"Failed to initialize Gemini RAG Agent: {e}")
    # The application can still run, but the /chat endpoint will fail.
    # A more robust solution might involve a health check endpoint.
    rag_agent = None


# --- Pydantic Models for FastAPI Request/Response Bodies ---

class RetrievalRequest(BaseModel):
    """
    Request model for the /retrieve endpoint.
    """
    query: str = Field(..., description="The natural language user query.")
    top_k: int = Field(5, ge=1, description="The number of top relevant text chunks to retrieve.")
    filters: Optional[Dict[str, Any]] = Field(None, description="Optional filters to apply to the retrieval query.")

class RetrievalResponseMetadata(BaseModel):
    """
    Metadata model for retrieved text chunks.
    """
    source_url: str = Field(..., description="URL of the page from which the content was extracted.")
    chapter_title: Optional[str] = Field(None, description="The title of the chapter this chunk belongs to.")
    module_name: Optional[str] = Field(None, description="The name of the module this chunk belongs to.")
    timestamp: Optional[str] = Field(None, description="The timestamp when this chunk was extracted and processed (ISO 8601 format).")

class RetrievalResponseChunk(BaseModel):
    """
    Response model for a single retrieved text chunk.
    """
    id: str = Field(..., description="Unique identifier of the text chunk.")
    content: str = Field(..., description="The textual content of the chunk.")
    metadata: RetrievalResponseMetadata = Field(..., description="Metadata associated with the text chunk.")

class ChatRequest(BaseModel):
    """
    Request model for the /chat endpoint.
    """
    query: str = Field(..., description="The user's question for the RAG agent.")

class ChatResponse(BaseModel):
    """
    Response model for the /chat endpoint.
    """
    answer: str = Field(..., description="The agent's generated answer.")


# --- API Endpoints ---

@app.post("/retrieve", response_model=List[RetrievalResponseChunk])
async def retrieve_chunks(request: RetrievalRequest):
    """
    Retrieves relevant text chunks from the vector database based on a query.
    This endpoint provides direct access to the retrieval component of the RAG pipeline.
    """
    try:
        retrieved_results = retrieve_context(
            query=request.query,
            top_k=request.top_k,
            filters=request.filters,
            collection_name="Physical Ai Book"
        )

        response_chunks = []
        for chunk in retrieved_results:
            metadata = RetrievalResponseMetadata(
                source_url=chunk.metadata.source_url,
                chapter_title=chunk.metadata.chapter_title,
                module_name=chunk.metadata.module_name,
                timestamp=chunk.metadata.timestamp,
            )
            response_chunks.append(RetrievalResponseChunk(
                id=chunk.id,
                content=chunk.content,
                metadata=metadata
            ))
        return response_chunks
    except Exception as e:
        logger.error(f"Error during retrieval: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e}")

@app.post("/chat") # Removed response_model=ChatResponse
async def chat_with_agent(request: ChatRequest): # Changed to async def
    """
    Interacts with the Gemini RAG agent to get a generated answer to a question.
    This endpoint uses the full RAG pipeline (Retrieve and Generate), returning a streaming response.
    """
    if not rag_agent:
        raise HTTPException(
            status_code=503, 
            detail="The RAG Agent is not available due to an initialization error. Please check the server logs."
        )
    
    async def event_generator():
        try:
            async for chunk in rag_agent.chat(request.query):
                # FastAPI expects byte strings for StreamingResponse
                yield f"data: {json.dumps({'content': chunk})}\n\n"
        except Exception as e:
            logger.error(f"Error during streaming chat with agent: {e}", exc_info=True)
            yield f"data: {json.dumps({'error': str(e)})}\n\n"
        finally:
            yield "event: end\ndata: {}\n\n" # Signal end of stream
            
    return StreamingResponse(event_generator(), media_type="text/event-stream")

@app.get("/health", status_code=200)
async def health_check():
    """
    Simple health check endpoint to confirm the API is running.
    """
    return {"status": "ok"}
