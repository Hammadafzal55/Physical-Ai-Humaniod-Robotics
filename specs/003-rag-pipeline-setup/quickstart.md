# Quickstart: RAG Pipeline Setup & Embedding Storage

This document provides a quick guide to understanding and interacting with the RAG pipeline's ingestion and retrieval components.

## 1. Overview

The RAG pipeline is responsible for extracting textbook content, generating vector embeddings using Cohere `embed-english-v3.0`, and storing them in Qdrant. A core part of this pipeline is the robust content extraction mechanism powered by Playwright.

## 2. Ingestion Process (Conceptual)

The ingestion process will involve:
1.  **Sitemap Discovery**: Identifying relevant URLs from the deployed frontend sitemap.
2.  **Page Content Extraction**: Using Playwright to render and extract text content from each URL.
3.  **Text Chunking**: Breaking down extracted text into manageable chunks suitable for embedding.
4.  **Embedding Generation**: Sending text chunks to Cohere's `embed-english-v3.0` model to generate vector representations.
5.  **Vector Storage**: Storing the generated vectors and associated metadata (e.g., `source_url`, `chapter`, `section`) in Qdrant.

## 3. Retrieval API (Conceptual)

The backend exposes a conceptual retrieval endpoint for agents to query the vector database.

**Endpoint**: `/retrieve` (POST)

**Request Body Example**:
```json
{
  "query_embedding": [...], // A list of floats representing the query's vector embedding
  "top_k": 5,              // Optional: Number of top results to return (default: 5)
  "filters": {             // Optional: Filters to apply to the search (e.g., by chapter)
    "chapter": "Introduction to Physical AI"
  }
}
```

**Response Body Example**:
```json
[
  {
    "id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
    "content": "...",
    "metadata": {
      "source_url": "https://example.com/...",
      "chapter": "...",
      "section": "...",
      "page_number": 10,
      "timestamp": "2025-12-21T10:30:00Z"
    }
  }
]
```

## 4. Key Technologies

*   **Content Extraction**: Playwright
*   **Embedding Model**: Cohere `embed-english-v3.0`
*   **Vector Database**: Qdrant
*   **Backend Framework**: FastAPI (for exposing retrieval endpoint)
*   **Package Manager**: `uv`

## 5. Development Notes

*   All backend logic resides in `backend/main.py`.
*   Ensure Playwright is correctly configured and its browser dependencies are available in the deployment environment.
*   Securely manage API keys for Cohere and Qdrant.
