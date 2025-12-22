from typing import List, Dict, Any, Optional

from pydantic import BaseModel, Field

# Reusing TextChunkMetadata from backend/main.py (or a compatible definition)
# For the purpose of this contract, we'll define a stripped-down version or refer to it.

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

# --- Function Signature (Conceptual) ---
# The core retrieval logic function would have a signature similar to this:
#
# def retrieve_context(
#     query: str,
#     top_k: int = 5,
#     filters: Optional[Dict[str, Any]] = None
# ) -> List[RetrievedChunk]:
#     """
#     Retrieves relevant textbook content from Qdrant for a given query.
#     """
#     pass # Implementation details in backend/retrieval.py
