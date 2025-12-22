# Research Findings: Retrieval Pipeline (Vector Search & Context Assembly)

## 1. Qdrant for Vector Similarity Search

*   **Decision**: Reuse the existing Qdrant instance and collection for vector similarity search.
*   **Rationale**: Qdrant was selected in Spec 3 for its efficiency, filtering capabilities, and Python client. It is already populated with textbook embeddings, making it the natural choice for retrieval. Its payload filtering will be crucial for any future metadata-based filtering.
*   **Alternatives Considered**: (None, as Qdrant is the established vector database for the project)

## 2. Cohere embed-english-v3.0 for Query Embeddings

*   **Decision**: Reuse Cohere `embed-english-v3.0` for generating embeddings of incoming user queries.
*   **Rationale**: To ensure semantic consistency and optimal similarity search results, the query embedding model *must* be the same as the model used to embed the stored documents. Cohere `embed-english-v3.0` was mandated and used in Spec 3.
*   **Alternatives Considered**: (None, as the model is mandated and consistency is key)

## 3. Context Assembly Best Practices

*   **Decision**: Implement context assembly by concatenating retrieved text chunks in order of their relevance score (highest first). Include source metadata (URL, chapter, module) for citation and verification.
*   **Rationale**: Ranking by relevance ensures the most pertinent information is presented first to the downstream agent. Including metadata provides essential grounding for the agent and allows users to trace the information back to its source.
*   **Alternatives Considered**:
    *   **Re-ranking**: Could use a cross-encoder model to re-rank initial retrieved chunks, but this adds complexity, latency, and cost, which may violate "low-end devices" constraint. Not pursued for initial implementation.
    *   **Context Window Optimization**: For agents with limited context windows, strategies like summarization or intelligent truncation of chunks could be applied. Not for initial implementation, but a future consideration.

## 4. Error Handling for Retrieval

*   **Decision**: Implement robust error handling for Qdrant and Cohere API calls, including graceful failure if no relevant data is found.
*   **Rationale**: Ensures a stable retrieval pipeline, preventing crashes and providing clear feedback to the downstream agent if a query cannot be fulfilled with relevant data. Retries for transient network issues will be used.
*   **Alternatives Considered**: (None, standard practice for API interactions)
