from typing import List, Dict, Any, Optional, Union
import os
import logging
from dotenv import load_dotenv
import qdrant_client
from qdrant_client.http.models import Distance, Filter, FieldCondition, MatchValue
import cohere
from cohere.core.api_error import ApiError

from pydantic import BaseModel, Field

# Load environment variables
load_dotenv()

# --- Configuration ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- Qdrant Client Initialization ---
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_HOST:
    raise ValueError("QDRANT_HOST environment variable not set.")

qdrant_client_instance = qdrant_client.QdrantClient(url=QDRANT_HOST, api_key=QDRANT_API_KEY, timeout=60)

# --- Cohere Client Initialization ---
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

if not COHERE_API_KEY:
    raise ValueError("COHERE_API_KEY environment variable not set.")

cohere_client = cohere.Client(COHERE_API_KEY)

# --- Pydantic Model Definitions ---
class RetrievalServiceMetadata(BaseModel):
    """
    Metadata for a retrieved text chunk, ready for agent consumption.
    Matches the metadata schema defined in Spec 3.
    """
    source_url: str = Field(..., description="URL of the source document.")
    chapter_title: Optional[str] = Field(None, description="Title of the chapter.")
    module_name: Optional[str] = Field(None, description="Name of the module.")
    timestamp: Optional[str] = Field(None, description="Timestamp of extraction (ISO 8601 format).")

class RetrievedChunk(BaseModel):
    """
    Represents a single text chunk retrieved from the vector database,
    along with its associated metadata and relevance score.
    """
    id: str = Field(..., description="Unique identifier of the text chunk.")
    content: str = Field(..., description="The textual content of the chunk.")
    metadata: RetrievalServiceMetadata = Field(..., description="Metadata associated with the text chunk.")
    relevance_score: float = Field(..., description="The similarity score from Qdrant, indicating relevance to the query.")

class RetrievalQueryParams(BaseModel):
    """
    Input parameters for the core retrieval function.
    """
    query: str = Field(..., description="The natural language user query.")
    top_k: int = Field(5, ge=1, description="The number of top relevant text chunks to retrieve.")
    filters: Optional[Dict[str, Any]] = Field(None, description="Optional filters to apply to the retrieval query (e.g., by chapter, source_url).")


# --- Retrieval Utility Functions ---
def _get_query_embedding(query: str) -> List[float]:
    """
    Generates an embedding for the given query using Cohere embed-english-v3.0.
    """
    if not query:
        logger.warning("Attempted to get embedding for an empty query.")
        return [] # Or raise an error
    
    logger.info(f"Generating embedding for query: '{query[:50]}...'")
    try:
        response = cohere_client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        logger.info("Successfully generated query embedding.")
        return response.embeddings[0]
    except ApiError as e:
        logger.error(f"Cohere API error during embedding generation for query '{query[:50]}...': {e}")
        raise RuntimeError(f"Cohere API error during embedding generation: {e}") from e
    except Exception as e:
        logger.error(f"Failed to generate query embedding for query '{query[:50]}...': {e}")
        raise RuntimeError(f"Failed to generate query embedding: {e}") from e

def _perform_vector_search(
    query_embedding: List[float],
    collection_name: str,
    top_k: int,
    filters: Optional[Dict[str, Any]] = None
) -> List[Any]: # Returns Qdrant search results
    """
    Performs a vector similarity search against the specified Qdrant collection.
    """
    logger.info(f"Performing vector search in collection '{collection_name}' for top {top_k} results.")
    try:
        # Construct Qdrant filter if provided
        qdrant_filter = None
        if filters:
            # Filter, FieldCondition, MatchValue are now imported at the top
            must_conditions = []
            for field, value in filters.items():
                must_conditions.append(FieldCondition(key=field, match=MatchValue(value=value)))
            qdrant_filter = Filter(must=must_conditions)
            logger.debug(f"Applying filters: {filters}")

        search_result = qdrant_client_instance.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k,
            query_filter=qdrant_filter,
            with_payload=True, # Ensure payload is returned for context assembly
        )
        logger.info(f"Vector search returned {len(search_result.points)} results.")
        return search_result.points
    except Exception as e:
        logger.error(f"Qdrant vector search failed in collection '{collection_name}': {e}")
        raise RuntimeError(f"Qdrant vector search failed: {e}") from e


def _assemble_context(retrieved_chunks: List[RetrievedChunk]) -> str:
    """
    Assembles a list of retrieved chunks into a single, formatted context string
    suitable for feeding into a large language model. Includes citations.
    """
    context_parts = []
    seen_sources = set()

    for i, chunk in enumerate(retrieved_chunks):
        # Add chunk content
        context_parts.append(f"--- Document {i+1} ---")
        context_parts.append(chunk.content)
        
        # Add citation if source hasn't been seen yet
        if chunk.metadata.source_url not in seen_sources:
            context_parts.append(f"Source: {chunk.metadata.source_url}")
            if chunk.metadata.chapter_title:
                context_parts.append(f"Chapter: {chunk.metadata.chapter_title}")
            seen_sources.add(chunk.metadata.source_url)
        context_parts.append("\n") # Separator between chunks

    return "\n".join(context_parts).strip()


# --- Main Retrieval Function ---
def retrieve_context(
    query: str,
    top_k: int = 5,
    filters: Optional[Dict[str, Any]] = None,
    collection_name: str = "Physical Ai Book", # Default to the collection created in Spec 3
    return_context_string: bool = False # New parameter to control return type
) -> Union[List[RetrievedChunk], str]: # Return type can be a list of chunks or a string
    """
    Retrieves relevant textbook content from Qdrant for a given query.
    If return_context_string is True, returns a single formatted string suitable for an LLM.
    """
    logger.info(f"Initiating retrieval for query: '{query[:50]}...' (top_k={top_k}, filters={filters})")
    if not query or not query.strip(): # Added check for whitespace-only queries
        logger.warning("Empty or whitespace-only query provided to retrieve_context.")
        return "" if return_context_string else []

    try:
        logger.debug("Generating query embedding.")
        query_embedding = _get_query_embedding(query)
        
        if not query_embedding:
            logger.warning("Query embedding could not be generated for retrieval.")
            return "" if return_context_string else []

        logger.debug("Performing vector search.")
        search_results = _perform_vector_search(query_embedding, collection_name, top_k, filters)

        if not search_results:
            logger.info(f"No relevant chunks found for query: '{query[:50]}...'")
            return "" if return_context_string else []

        retrieved_chunks = []
        for hit in search_results:
            payload = hit.payload
            metadata = RetrievalServiceMetadata(
                source_url=payload.get("source_url", ""),
                chapter_title=payload.get("chapter_title"),
                module_name=payload.get("module_name"),
                timestamp=payload.get("timestamp"),
            )
            retrieved_chunks.append(RetrievedChunk(
                id=str(hit.id),
                content=payload.get("content", ""),
                metadata=metadata,
                relevance_score=hit.score
            ))
        
        logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: '{query[:50]}...'.")
        if return_context_string:
            assembled_context = _assemble_context(retrieved_chunks)
            logger.debug("Assembled context string.")
            return assembled_context
        else:
            return retrieved_chunks
    except Exception as e:
        logger.error(f"Error during retrieval for query '{query[:50]}...': {e}", exc_info=True)
        # Graceful failure for retrieval
        return "" if return_context_string else [] # Return empty list/string on failure