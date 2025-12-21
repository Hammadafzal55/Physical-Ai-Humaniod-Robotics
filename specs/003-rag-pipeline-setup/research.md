# Research: RAG Pipeline Setup & Embedding Storage

## Decision: Content Extraction Strategy from Deployed Docusaurus URLs

**Rationale**: Extracting content directly from deployed Docusaurus URLs ensures the RAG pipeline processes the same content visible to users. Python libraries like `BeautifulSoup` or `requests-html` can be used to fetch and parse the HTML content, extracting the main text. This avoids needing direct filesystem access to the Docusaurus build output or Markdown source files, making the pipeline deployment-agnostic.

**Alternatives Considered**:
-   **Directly parsing Docusaurus Markdown files**: Rejected because it would tightly couple the backend to the frontend's repository structure and require access to the source files, which might not be available in a deployed scenario.
-   **Using Docusaurus API (if any)**: Docusaurus primarily generates static HTML; a dedicated content API is not a native feature. Custom solutions would add unnecessary complexity.

## Decision: Text Chunking Strategy for RAG Pipelines

**Rationale**: Effective text chunking is crucial for RAG performance, balancing context preservation with embedding model token limits. A common strategy involves splitting text into paragraphs or sentences and then grouping them into larger chunks that fit within the Cohere `embed-english-v3.0` model's token limit (e.g., 512 tokens). Overlapping chunks can improve retrieval recall. Metadata (chapter, module, URL) should be preserved with each chunk.

**Alternatives Considered**:
-   **Fixed-size character chunking**: Can break sentences or paragraphs, losing semantic context.
-   **Sentence-based chunking without grouping**: May result in too many small chunks, leading to less contextual embeddings and higher Qdrant storage/query costs.
-   **Large, non-overlapping chunks**: Risk exceeding token limits and losing fine-grained retrieval ability.

## Decision: Optimal Qdrant Collection Setup for Textbook Content

**Rationale**: A Qdrant collection should be created with a vector size matching Cohere `embed-english-v3.0`'s output dimension (typically 1024). The `Cosine` distance metric is standard for semantic similarity. Metadata (source URL, chapter title, module name, original text) should be indexed as payload fields to facilitate filtering and contextual retrieval. The original text content should also be stored for direct use after retrieval.

**Alternatives Considered**:
-   **Different distance metrics (e.g., Euclidean)**: Cosine similarity is generally more suitable for high-dimensional embeddings and text similarity.
-   **No metadata indexing**: Would make filtering or contextual awareness during retrieval much harder or impossible.

## Decision: Error Handling and Retry Mechanisms for External APIs (Cohere, Qdrant)

**Rationale**: Network calls to external services like Cohere API and Qdrant client are prone to transient failures (e.g., rate limits, network issues). Implementing retry logic with exponential backoff (e.g., using a library like `tenacity`) for these API calls enhances the pipeline's robustness and resilience. Specific error logging should be included to diagnose persistent issues.

**Alternatives Considered**:
-   **No retry mechanism**: Leads to brittle pipelines that fail on transient network issues.
-   **Fixed delay retries**: Less efficient than exponential backoff, which adapts to API responsiveness.

## Decision: `uv package` Project Initialization and Dependency Management

**Rationale**: The `uv package` is a modern, fast Python package installer and resolver. It should be used to initialize the backend project, manage dependencies (`pyproject.toml`), and create virtual environments. This aligns with the architecture principles for a lean and efficient Python backend.

**Alternatives Considered**:
-   **`pip` and `venv`**: `uv` offers significantly faster dependency resolution and installation, making it a preferred choice for development efficiency.
-   **`Poetry` or `PDM`**: While robust, `uv` is explicitly specified by the architecture principles and provides a lighter-weight, high-performance alternative suitable for this project's scale.
