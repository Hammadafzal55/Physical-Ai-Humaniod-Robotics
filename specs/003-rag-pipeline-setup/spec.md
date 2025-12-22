## Spec 3: RAG Pipeline Setup & Embedding Storage

### 1. Overview

This specification outlines the development of a Retrieval-Augmented Generation (RAG) pipeline for an AI-native textbook and chatbot system. The primary goal is to extract content from the textbook, generate embeddings using the Cohere `embed-english-v3.0` model, and store these embeddings in a Qdrant vector database. The backend code will be scaffolded to support future RAG retrieval by the agent, with Playwright integrated as a robust fallback for content extraction.

### 2. Goals

*   Build a RAG pipeline that extracts content from the textbook, creates embeddings, and stores them in a vector database.
*   Utilize the Cohere embedding model `embed-english-v3.0` for generating embeddings.
*   Store generated vectors in Qdrant for efficient retrieval.
*   Scaffold backend code to enable subsequent RAG retrieval functionality by the agent.
*   Implement Playwright as a fallback mechanism to ensure robust content extraction if traditional scraping methods fail.

### 3. Non-Goals

*   Implementation of the RAG chatbot interaction logic.
*   Deployment of the Qdrant vector database (assumed to be externally managed or provided).
*   Development of a user interface for managing the RAG pipeline.
*   Real-time processing of content changes (initial implementation focuses on batch ingestion).

### 4. User Scenarios & Testing

**User Role:** System Administrator / Automated Content Ingestion Process

**Scenario 1: Successful Textbook Content Ingestion and Embedding**
*   **Given** accessible deployment URLs for the frontend textbook content.
*   **When** the RAG pipeline is triggered for content ingestion.
*   **Then** all specified textbook content is successfully extracted.
*   **And** embeddings are generated for the extracted content using the Cohere `embed-english-v3.0` model.
*   **And** these embeddings, along with relevant metadata, are stored in the Qdrant vector database.
*   **And** the backend exposes a functional interface for RAG retrieval.

**Scenario 2: Robust Content Extraction using Playwright Fallback**
*   **Given** an attempt to extract content from a textbook URL fails using the primary extraction method.
*   **When** Playwright is invoked as the fallback content extraction mechanism.
*   **Then** Playwright successfully extracts the content from the problematic URL.
*   **And** the extracted content proceeds through the embedding generation and storage steps as in Scenario 1.

### 5. Functional Requirements

*   **FR1: Content Extraction:** The pipeline SHALL extract content from a list of provided frontend textbook URLs.
    *   **FR1.1:** The pipeline SHALL support a primary content extraction method.
    *   **FR1.2:** The pipeline SHALL implement Playwright as a fallback mechanism for content extraction when the primary method fails.
*   **FR2: Embedding Generation:** The pipeline SHALL generate embeddings for extracted text chunks using the Cohere `embed-english-v3.0` model.
*   **FR3: Vector Storage:** The pipeline SHALL store the generated embeddings and associated metadata (e.g., source URL, chapter, section) in a Qdrant vector database collection.
*   **FR4: Backend Scaffolding for Retrieval:** The backend application SHALL expose an API endpoint or function that allows agents to query the Qdrant database for relevant text chunks based on a given query embedding.

### 6. Non-Functional Requirements

*   **Performance:**
    *   **NFR1.1:** Content extraction for a single textbook page SHALL complete within 30 seconds.
    *   **NFR1.2:** Embedding generation for a typical text chunk (e.g., 500 tokens) SHALL complete within 5 seconds.
    *   **NFR1.3:** Storage of an embedding and its metadata in Qdrant SHALL complete within 100 milliseconds.
*   **Scalability:** The RAG pipeline SHALL be designed to accommodate future growth in textbook content volume and number of URLs without significant architectural changes.
*   **Reliability:** The content extraction process SHALL be resilient to common web scraping challenges (e.g., dynamic content, anti-bot measures) through the use of Playwright as a fallback.
*   **Maintainability:** The backend code SHALL adhere to established coding standards, be modular, and well-documented to facilitate future enhancements and integration by the agent.
*   **Security:** Access to the Cohere API and Qdrant database SHALL be managed securely, utilizing environment variables or a secrets management system.

### 7. Data Model/Key Entities

*   **Text Chunk:**
    *   `id`: Unique identifier (UUID).
    *   `content`: The textual content extracted from the textbook.
    *   `metadata`:
        *   `source_url`: URL of the page from which the content was extracted.
        *   `chapter`: (Optional) Chapter title.
        *   `section`: (Optional) Section title.
        *   `page_number`: (Optional) Page number.
        *   `timestamp`: Time of extraction/embedding.
*   **Embedding:**
    *   `vector`: Numerical representation (list of floats) generated by Cohere.
*   **Qdrant Collection:**
    *   A dedicated collection in Qdrant will store Text Chunk IDs, embeddings, and metadata.

### 8. Technical Considerations/Assumptions

*   Frontend textbook URLs will be stable and consistently available for extraction.
*   The `uv package` in the `backend` folder is fully initialized, and all necessary Python dependencies are already installed and managed.
*   Access to the Cohere embedding model API (`embed-english-v3.0`) is configured and authenticated.
*   A Qdrant instance is accessible via a network endpoint with appropriate authentication.
*   The core pipeline logic will reside in a single modular Python file, likely `main.py`, within the `backend` directory.
*   Playwright will be installed and configured in the backend environment for fallback web scraping.

### 9. Success Criteria

*   100% of specified textbook content URLs are successfully processed, resulting in valid embeddings stored in Qdrant.
*   The Qdrant collection contains embeddings and metadata for all extracted content, allowing for efficient vector search.
*   The scaffolded backend retrieval interface returns relevant text chunks when queried with appropriate embeddings.
*   The Playwright fallback mechanism is demonstrably effective in extracting content from pages where the primary method fails.
*   The backend codebase is clean, modular, and adheres to the `uv package` structure, ready for further agent integration.
