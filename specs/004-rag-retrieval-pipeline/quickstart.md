# Quickstart: Retrieval Pipeline (Vector Search & Context Assembly)

This document provides a quick guide to understanding and interacting with the RAG retrieval pipeline.

## 1. Overview

The retrieval pipeline is responsible for accepting a user query, finding the most relevant textbook content from the Qdrant vector database (populated by Spec 3), and assembling this into a structured, citation-ready context.

## 2. Core Retrieval Process (Conceptual)

The retrieval process will involve:
1.  **Query Input**: Receiving a natural language user query.
2.  **Query Embedding**: Generating a vector embedding for the query using Cohere `embed-english-v3.0`.
3.  **Vector Search**: Performing a similarity search against the existing Qdrant collection (`Physical Ai Book`).
4.  **Top-K Retrieval**: Retrieving the top-K most relevant document chunks.
5.  **Context Assembly**: Structuring the retrieved chunks, including their text, source URL, and module/chapter metadata, into a format suitable for downstream agents.

## 3. Retrieval Service (Conceptual)

The core retrieval logic will be encapsulated in a function within `backend/retrieval.py`. This function will be callable by other backend components (e.g., a FastAPI endpoint).

**Function Signature (conceptual)**:
```python
def retrieve_context(
    query: str,
    top_k: int = 5,
    filters: Optional[Dict[str, Any]] = None
) -> List[RetrievedChunk]:
    """
    Retrieves relevant textbook content from Qdrant for a given query.
    """
    # Implementation will be in backend/retrieval.py
    pass
```

## 4. Key Technologies

*   **Embedding Model**: Cohere `embed-english-v3.0` (for query embedding)
*   **Vector Database**: Qdrant (for storage and similarity search)
*   **Backend Language**: Python 3.11+
*   **File Location**: `backend/retrieval.py`

## 5. Development Notes

*   Ensure Cohere API key and Qdrant connection details are correctly configured (reusing settings from Spec 3).
*   The retrieval logic should be deterministic and return results ranked by relevance.
*   The output format for `RetrievedChunk` should align with the defined contract (`retrieval_service_contract.py`).
