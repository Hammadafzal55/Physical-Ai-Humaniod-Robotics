---

description: "Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts."
---

# Tasks: Phase 1 — Spec 3: RAG Pipeline Setup & Embedding Storage

**Input**: Design documents from `/specs/003-rag-pipeline-setup/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below are relative to the project root.

## Phase 1: Setup

**Purpose**: Initialize the backend project and install core dependencies.

-   [ ] T001 Navigate to the `backend/` directory.
-   [ ] T002 Initialize the Python project using `uv` (create virtual environment, `pyproject.toml` if not exists). (`backend/`)
-   [ ] T003 Install core dependencies using `uv` for FastAPI, Cohere, Qdrant-client, requests-html, beautifulsoup4, python-dotenv, uvicorn, tenacity. (`backend/pyproject.toml`)

## Phase 2: Foundational

**Purpose**: Establish the basic structure for the RAG pipeline.

-   [ ] T004 Create the `main.py` file for the RAG pipeline. (`backend/main.py`)
-   [ ] T005 Set up environment variables in a `.env` file in the `backend/` directory for `COHERE_API_KEY`, `QDRANT_HOST`, `QDRANT_API_KEY`, and `TEXTBOOK_URLS`. (`backend/.env`)

## Phase 3: User Story 1 - Textbook Content Extraction & Embedding (Priority: P1)

**Goal**: As a system administrator, I need to extract all relevant content from the deployed Docusaurus textbook, generate embeddings for each meaningful chunk of text using Cohere `embed-english-v3.0`, and store these embeddings, along with their original text and metadata, into a Qdrant vector database. This will enable the RAG chatbot to retrieve relevant information from the textbook.

**Independent Test**: Can be fully tested by running the RAG pipeline script against the deployed frontend URLs, verifying that the Qdrant database is populated with a sufficient number of embeddings, and that a sample query returns relevant text chunks.

### Implementation for User Story 1

-   [ ] T006 [P] [US1] Implement `extract_text_from_url(url: str)` function to fetch and parse HTML content from a given URL, returning clean text. (`backend/main.py`)
-   [ ] T007 [P] [US1] Implement `chunk_text(text: str)` function to split long texts into semantically meaningful chunks, adhering to Cohere token limits and including overlap. (`backend/main.py`)
-   [ ] T008 [P] [US1] Implement `embed(texts: List[str])` function to generate embeddings for a list of text chunks using Cohere `embed-english-v3.0`. (`backend/main.py`)
-   [ ] T009 [P] [US1] Implement `create_collection(collection_name: str, vector_size: int, distance_metric: str)` function to initialize a Qdrant collection with specified parameters. (`backend/main.py`)
-   [ ] T010 [P] [US1] Implement `save_chunk_to_qdrant(collection_name: str, text_chunk: TextChunk, embedding: Embedding)` function to store an embedding and its metadata in Qdrant. (`backend/main.py`)
-   [ ] T011 [P] [US1] Implement `ingest_book(textbook_urls: List[str], collection_name: str)` function that orchestrates the entire pipeline:
    1.  Fetches URLs.
    2.  Extracts content.
    3.  Chunks text.
    4.  Generates embeddings.
    5.  Stores in Qdrant. (`backend/main.py`)
-   [ ] T012 [P] [US1] Implement a utility function (e.g., `get_textbook_chapter_urls`) to fetch a list of deployed Docusaurus chapter URLs dynamically or from a predefined list. (`backend/main.py`)

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the pipeline with robustness and validation.

-   [ ] T013 Implement error handling and retry mechanisms for Cohere API and Qdrant client interactions, including exponential backoff. (`backend/main.py`)
-   [ ] T014 Add logging for pipeline progress, errors, and warnings. (`backend/main.py`)
-   [ ] T015 Write a simple validation script or function (e.g., `verify_qdrant_data`) to query Qdrant and verify ingested data, matching `SC-004`. (`backend/main.py`)
-   [ ] T016 Implement main execution logic to run the ingestion pipeline when `main.py` is executed directly. (`backend/main.py`)

---
## Dependencies & Execution Order

### Phase Dependencies

-   Phase 1 (Setup) must complete before Phase 2 (Foundational).
-   Phase 2 (Foundational) must complete before Phase 3 (User Story 1).
-   Phase 3 (User Story 1) must be substantially complete before Phase 4 (Polish & Cross-Cutting Concerns) begins.

### User Story Dependencies

-   User Story 1 (P1) is independent, but its tasks have internal dependencies.

### Within Each User Story

-   Implementation tasks (T006-T012) are logically ordered to minimize dependencies. T011 depends on T006-T010. T016 depends on T011 and T012.
-   Tests (if generated) for a specific task should run before its implementation.

### Parallel Opportunities

-   Within Phase 3, tasks T006-T010 (individual utility functions) can be developed in parallel, but their integration into T011 (ingest_book) is sequential.
-   T013 (Error Handling) and T014 (Logging) can be integrated progressively throughout the implementation of T006-T012.
-   T015 (Validation Script) can be developed in parallel with T006-T012 once basic ingestion is functional.

## Implementation Strategy

### MVP First (Functional Ingestion Pipeline)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete core functions of Phase 3 (T006, T007, T008, T009, T010, T012) to create an ingest_book function.
4.  Execute T011 (`ingest_book`) to verify basic data ingestion into Qdrant.
5.  **STOP and VALIDATE**: Verify that the Qdrant database is populated with basic embeddings.

### Incremental Delivery

1.  Implement and thoroughly test each utility function (T006-T010) individually.
2.  Integrate these functions into the main `ingest_book` orchestrator (T011).
3.  Progressively add error handling (T013) and logging (T014).
4.  Develop the validation script (T015) to confirm data integrity.
5.  Implement the main execution logic (T016).

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
