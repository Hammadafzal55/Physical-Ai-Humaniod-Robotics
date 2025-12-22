# Feature Specification: Retrieval Pipeline (Vector Search & Context Assembly)

**Feature Branch**: `main`  
**Created**: 2025-12-22  
**Status**: Draft  
**Input**: User description: "SPEC NAME: Spec 4: Retrieval Pipeline (Vector Search & Context Assembly) CONTEXT: - Project: AI-native textbook + RAG chatbot system. - Spec 3 (RAG Pipeline Setup & Embedding Storage) is **completed**. - In Spec 3: - Textbook content was discovered via sitemap → URLs - Pages were fetched using **mandatory Playwright-based scraping** - Content was cleaned, chunked, embedded using **Cohere `embed-english-v3.0`** - Embeddings were stored in **Qdrant** - All ingestion logic lives in `backend/main.py` - All work was done strictly on the **main branch** This spec builds **directly on top of Spec 3 outputs** and MUST NOT re-ingest, re-scrape, or re-embed content. OBJECTIVE: Build a reliable retrieval pipeline that: - Queries Qdrant using vector similarity - Retrieves the most relevant textbook chunks - Assembles clean, ordered context - Prepares retrieved data for downstream agent usage (Spec 5) SCOPE: Includes: - Connecting to the existing Qdrant collection created in Spec 3 - Accepting a user query as input - Generating a query embedding using **Cohere `embed-english-v3.0`** - Performing similarity search against stored vectors - Retrieving top-K relevant chunks - Returning structured, citation-ready context (text + metadata) ARCHITECTURE & FILE RULES: - Backend code lives in `/backend` - Retrieval logic MUST be implemented in: - `retrieval.py` - `main.py` MUST NOT be modified unless absolutely required - Retrieval code must reuse: - Qdrant collection - Embedding dimensions - Metadata schema defined in Spec 3 - No duplication of embedding or ingestion logic RETRIEVAL REQUIREMENTS: - Retrieval must be deterministic and testable - Results must be ranked by relevance score - Returned chunks must include: - Chunk text - Source URL - Module / chapter metadata - Context must be safe for direct consumption by an agent - Retrieval must fail gracefully if no relevant data is found QUALITY CONSTRAINTS: - Retrieval must ONLY return textbook-derived data - No hallucinated or external content - Chunk ordering and cleanliness must be preserved - Performance must be suitable for low-end devices GIT & WORKFLOW RULES: - All work MUST remain on the **main branch** - ❌ No new branches - ❌ No branch switching - ❌ No rewriting Spec 3 code unless strictly required DEFINITION OF DONE: - A working retrieval pipeline implemented in `retrieval.py` - Queries correctly return relevant textbook chunks from Qdrant - Output is structured and ready for agent consumption - Spec 4 fully aligned with decisions and constraints from Spec 3"

## User Scenarios & Testing

### User Story 1 - Retrieve Relevant Context for User Query (Priority: P1)

The system needs to efficiently find and present the most relevant information from the ingested textbook content when a user asks a question. This directly supports the RAG chatbot's ability to answer questions accurately.

**Why this priority**: This is the core functionality of the retrieval pipeline and is critical for the subsequent RAG agent (Spec 5) to function. Without it, the chatbot cannot retrieve relevant context.

**Independent Test**: The retrieval pipeline can be tested by providing a natural language query and verifying that it returns a list of textbook chunks that are highly relevant to the query, correctly ordered by relevance, and include all specified metadata (text, source URL, module/chapter).

**Acceptance Scenarios**:

1.  **Given** a user provides a natural language query, **When** the retrieval pipeline is invoked with this query, **Then** it generates an embedding for the query using Cohere `embed-english-v3.0`.
2.  **Given** a query embedding, **When** the retrieval pipeline performs a similarity search against the Qdrant collection, **Then** it retrieves the top-K most relevant textbook chunks.
3.  **Given** retrieved top-K chunks, **When** the pipeline assembles the context, **Then** it returns a structured output containing the chunk text, source URL, and module/chapter metadata for each relevant chunk.
4.  **Given** a query for which no relevant chunks are found in Qdrant, **When** the retrieval pipeline is invoked, **Then** it gracefully indicates that no relevant data was found (e.g., by returning an empty list or a specific status).

### User Story 2 - Robustness and Quality of Retrieved Context (Priority: P2)

The system must ensure that the retrieved context is accurate, directly from the textbook, well-ordered, and clean, ready for consumption by the downstream RAG agent.

**Why this priority**: This ensures the quality and reliability of the RAG agent's responses, preventing hallucinations and providing trustworthy information.

**Independent Test**: The retrieval pipeline can be tested with various queries, including those designed to probe for external content or incorrect ordering, and verifying that only textbook-derived content is returned, ordering is maintained, and the output format is consistent and clean.

**Acceptance Scenarios**:

1.  **Given** a query, **When** the retrieval pipeline returns a context, **Then** all returned chunks consist ONLY of textbook-derived data, with no external or hallucinated content.
2.  **Given** multiple relevant chunks from the same source, **When** they are assembled into context, **Then** their original ordering from the textbook is preserved as much as possible to maintain coherence.
3.  **Given** the output from the retrieval pipeline, **When** it is presented to a downstream RAG agent, **Then** the context is clean, well-formatted, and safe for direct consumption.

### Edge Cases

-   What happens when the Qdrant collection is empty or unreachable?
-   How does the system handle very short queries or queries with no semantic match?
-   What if the Cohere API fails during query embedding generation?

## Requirements

### Functional Requirements

-   **FR-001: Qdrant Connection**: The retrieval pipeline SHALL connect to the existing Qdrant collection created in Spec 3.
-   **FR-002: Query Input**: The retrieval pipeline SHALL accept a natural language user query (string) as input.
-   **FR-003: Query Embedding**: The retrieval pipeline SHALL generate an embedding for the user query using Cohere `embed-english-v3.0`.
-   **FR-004: Vector Search**: The retrieval pipeline SHALL perform a vector similarity search against the stored embeddings in Qdrant.
-   **FR-005: Top-K Retrieval**: The retrieval pipeline SHALL retrieve the top-K (configurable) most relevant textbook chunks, ranked by relevance score.
-   **FR-006: Context Assembly**: The retrieval pipeline SHALL assemble the retrieved chunks into a clean and ordered context suitable for agent consumption.
-   **FR-007: Structured Output**: The retrieval pipeline SHALL return structured output for each retrieved chunk, including:
    *   Chunk text
    *   Source URL
    *   Module / chapter metadata
-   **FR-008: Graceful Failure**: The retrieval pipeline SHALL fail gracefully if no relevant data is found in Qdrant for a given query (e.g., by returning an empty list or specific status).

### Key Entities

-   **User Query**: The natural language text input from the user.
-   **Query Embedding**: The vector representation of the User Query, generated by Cohere `embed-english-v3.0`.
-   **Retrieved Chunk**: A chunk of text retrieved from Qdrant, along with its associated metadata and a relevance score.
    *   `text`: The content of the chunk.
    *   `source_url`: URL of the source document.
    *   `chapter_title`: Title of the chapter.
    *   `module_name`: Name of the module.
    *   `relevance_score`: The similarity score from Qdrant.

## Success Criteria

### Measurable Outcomes

-   **SC-001**: A working retrieval pipeline is implemented in `backend/retrieval.py` and can be successfully invoked with a user query.
-   **SC-002**: For any given query, the pipeline correctly returns the top-K relevant textbook chunks from the Qdrant collection.
-   **SC-003**: The output for retrieved chunks is consistently structured, containing chunk text, source URL, and module/chapter metadata, ready for direct consumption by a downstream RAG agent.
-   **SC-004**: Retrieval latency for 95% of queries is under 5 seconds, even on low-end devices.
-   **SC-005**: All retrieved content is verifiable as originating solely from the textbook, with no extraneous or hallucinated information.
-   **SC-006**: The pipeline gracefully handles scenarios where no relevant data is found, without crashing.
