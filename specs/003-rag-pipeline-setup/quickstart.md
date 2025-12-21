# Quickstart: RAG Pipeline Setup & Embedding Storage

This guide provides a quick overview of how to set up and run the RAG pipeline for extracting textbook content, generating embeddings, and storing them in Qdrant.

## 1. Prerequisites

- Python 3.11+ installed.
- Access to a Qdrant instance (local or Qdrant Cloud).
- A Cohere API key with access to `embed-english-v3.0`.
- Deployed Docusaurus frontend URLs for the textbook content.

## 2. Backend Setup

1.  **Navigate to the `backend` directory**:
    ```bash
    cd backend
    ```

2.  **Initialize `uv` project and install dependencies**:
    If `uv` is not installed, install it: `pip install uv`.
    ```bash
    uv venv
    uv pip install -r requirements.txt # Assuming requirements.txt will be generated
    # Or, if using pyproject.toml directly:
    uv pip install .
    ```
    Alternatively, for a fresh setup:
    ```bash
    uv init
    # Add dependencies to pyproject.toml
    uv pip install cohere qdrant-client fastapi uvicorn beautifulsoup4 requests-html python-dotenv
    ```

3.  **Configure Environment Variables**:
    Create a `.env` file in the `backend/` directory with the following:
    ```dotenv
    COHERE_API_KEY="YOUR_COHERE_API_KEY"
    QDRANT_HOST="YOUR_QDRANT_HOST" # e.g., "localhost" or your Qdrant Cloud URL
    QDRANT_API_KEY="YOUR_QDRANT_API_KEY" # Only for Qdrant Cloud or authenticated instances
    TEXTBOOK_URLS="URL1,URL2,URL3" # Comma-separated deployed Docusaurus chapter URLs
    ```
    Replace placeholders with your actual keys and URLs.

## 3. Running the RAG Pipeline (Embedding & Ingestion)

The RAG pipeline logic for embedding and ingestion resides in `backend/main.py`.

1.  **Ensure Qdrant is running**:
    If using a local Qdrant instance, make sure it's active.
2.  **Execute the ingestion script**:
    ```bash
    uv run python main.py # This will execute the ingestion logic within main.py
    ```
    This script will:
    -   Fetch content from the specified `TEXTBOOK_URLS`.
    -   Chunk the text.
    -   Generate embeddings using Cohere.
    -   Store embeddings and metadata in Qdrant.

## 4. Verifying Data in Qdrant

After running the ingestion script, you can verify that the data has been stored in your Qdrant instance.

-   **Qdrant UI**: Access your Qdrant instance's web UI (if available) to browse the collections and points.
-   **Qdrant Client**: Use the Qdrant client in a Python script to connect to your instance and perform a sample search:
    ```python
    from qdrant_client import QdrantClient
    from cohere import Client as CohereClient
    import os

    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    qdrant_client = QdrantClient(
        host=os.getenv("QDRANT_HOST"),
        api_key=os.getenv("QDRANT_API_KEY") # Only if authenticated
    )
    cohere_client = CohereClient(os.getenv("COHERE_API_KEY"))

    collection_name = "textbook_embeddings"
    query_text = "What are the main components of ROS 2?"

    # Generate embedding for the query
    query_embedding = cohere_client.embed(
        texts=[query_text],
        model="embed-english-v3.0",
        input_type="search_query"
    ).embeddings[0]

    # Search Qdrant
    search_result = qdrant_client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        limit=3, # Retrieve top 3
        with_payload=True
    )

    print(f"Top 3 results for query: '{query_text}'")
    for hit in search_result:
        print(f"Score: {hit.score}")
        print(f"Text: {hit.payload['content'][:200]}...") # Print first 200 chars
        print(f"Source: {hit.payload['source_url']}")
        print(f"Chapter: {hit.payload['chapter_title']}\n")
    ```

This quickstart guides you through the process of setting up and populating your Qdrant database with textbook embeddings, ready for retrieval in subsequent phases.
