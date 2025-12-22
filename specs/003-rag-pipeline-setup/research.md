# Research Findings: RAG Pipeline Setup & Embedding Storage

## 1. Playwright for Robust Content Extraction

*   **Decision**: Utilize Playwright for all web page rendering and content extraction.
*   **Rationale**: Playwright provides capabilities for handling dynamic content, JavaScript execution, and CAPTCHAs/anti-bot measures effectively. This addresses the constraint of Vercel blocking standard scraping and ensures reliable content acquisition. Its API allows for precise element selection and extraction, and can handle various page states (e.g., waiting for network requests).
*   **Alternatives Considered**:
    *   `requests` + `BeautifulSoup`: Rejected due to inability to handle JavaScript-rendered content and high likelihood of being blocked.
    *   `Scrapy`: More robust than `requests`/`BeautifulSoup` but still struggles with complex JavaScript rendering without additional tools, and adds significant overhead for this specific use case.

## 2. Cohere embed-english-v3.0 for Embeddings

*   **Decision**: Directly integrate with the Cohere API using their Python SDK for `embed-english-v3.0`.
*   **Rationale**: The spec explicitly mandates this model. The Cohere Python SDK provides a straightforward interface for text embedding, handling API requests, and managing rate limits.
*   **Alternatives Considered**: (None, as the model is mandated)

## 3. Qdrant for Vector Database Storage

*   **Decision**: Use Qdrant (self-hosted or free-tier cloud) for vector storage.
*   **Rationale**: Qdrant offers efficient similarity search, robust filtering capabilities, and a well-documented Python client. It supports storing vectors with associated payloads (metadata), which is crucial for contextual retrieval. Its open-source nature and free-tier cloud option align with project constraints.
*   **Alternatives Considered**:
    *   `Pinecone`: Also a strong contender, but Qdrant's local deployment option and open-source nature provides more flexibility and aligns well with potential free-tier cloud usage.
    *   `Weaviate`: Similar to Qdrant, but Qdrant's Python client and payload filtering capabilities were deemed slightly more aligned for this project's needs.

## 4. uv package management & Backend structure

*   **Decision**: Maintain the existing `uv` package structure and ensure all new pipeline logic resides within `backend/main.py`.
*   **Rationale**: The project explicitly states `uv` is initialized and dependencies installed. Centralizing pipeline logic in `main.py` adheres to the "single modular file" principle, simplifies debugging, and streamlines future agent integration. `uv` will manage Python dependencies efficiently.
*   **Alternatives Considered**:
    *   Distributing logic across multiple files in `backend/`: Rejected due to the explicit "single modular file (e.g., `main.py`)" architecture principle for this phase, as stated in the spec.
