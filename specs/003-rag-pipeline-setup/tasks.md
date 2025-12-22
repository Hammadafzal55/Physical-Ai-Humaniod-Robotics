---

description: "Task list for RAG Pipeline Setup & Embedding Storage"
---

# Tasks: RAG Pipeline Setup & Embedding Storage

**Input**: Design documents from `specs/003-rag-pipeline-setup/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification does not explicitly request test tasks, so none will be generated.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below assume single project - adjust based on plan.md structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic environment configuration for Playwright, Cohere, and Qdrant.

- [X] T001 Install Playwright browser binaries and dependencies. (This usually involves a shell command like `playwright install` in the environment).
- [X] T002 Configure environment variables for Cohere API key in `.env` file.
- [X] T003 Configure environment variables for Qdrant connection details (URL, API key) in `.env` file.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Implement core data models and client initializations that are prerequisites for any user story.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Define `TextChunk` Pydantic model with `id`, `content`, and `metadata` fields in `backend/main.py`.
- [X] T005 Initialize Qdrant client within `backend/main.py`.
- [X] T006 Implement function to create Qdrant collection, including vector size and distance metric, in `backend/main.py`.
- [X] T007 Initialize Cohere client within `backend/main.py`.
- [X] T008 Implement text chunking logic (e.g., splitting text into fixed-size chunks) in `backend/main.py`.

---

## Phase 3: User Story 1 - Successful Textbook Content Ingestion and Embedding (Priority: P1) 🎯 MVP

**Goal**: Implement the core ingestion pipeline: URL discovery, content extraction using Playwright (primary method), embedding generation with Cohere, Qdrant storage, and scaffolding for the retrieval endpoint.

**Independent Test**: A given list of textbook URLs can be processed, and their content correctly extracted, embedded, and stored in Qdrant with associated metadata. The backend exposes a functional interface for RAG retrieval.

### Implementation for User Story 1

- [X] T009 [US1] Implement URL discovery function (e.g., `get_sitemap_urls`) in `backend/main.py`.
- [X] T010 [P] [US1] Implement primary content extraction function using Playwright (e.g., `extract_content_playwright`) in `backend/main.py`.
- [X] T011 [P] [US1] Implement embedding generation function using Cohere API (e.g., `generate_embedding_cohere`) in `backend/main.py`.
- [X] T012 [P] [US1] Implement Qdrant storage function (e.g., `store_chunks_in_qdrant`) to upsert vectors and payloads in `backend/main.py`.
- [X] T013 [US1] Integrate extraction, chunking, embedding, and storage into a main ingestion pipeline function (e.g., `run_ingestion_pipeline`) in `backend/main.py`.
- [X] T014 [US1] Scaffold FastAPI `POST /retrieve` endpoint in `backend/main.py` based on `retrieval_contract.json` to allow agents to query Qdrant.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Robust Content Extraction with Playwright Fallback (Priority: P2)

**Goal**: Enhance the content extraction mechanism to include Playwright as a fallback when primary extraction methods (if any other were to be implemented later) fail. For this phase, Playwright is the primary method, so the fallback logic will ensure Playwright itself is robust.

**Independent Test**: When the initial content extraction attempt (e.g., if it times out or returns empty content unexpectedly) fails for a given URL, the Playwright fallback mechanism is successfully re-attempted or used as the robust method, and the content is extracted, processed, embedded, and stored.

### Implementation for User Story 2

- [X] T015 [US2] Refine `extract_content_playwright` function in `backend/main.py` to include robust error handling, retries, and explicit fallback logic within Playwright (e.g., different selectors, waiting strategies).
- [X] T016 [US2] Integrate the refined `extract_content_playwright` into the main ingestion pipeline.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and overall system quality.

- [X] T017 Implement comprehensive error handling and exception management across the ingestion pipeline in `backend/main.py`.
- [X] T018 Integrate structured logging for key operations (e.g., page extraction, embedding generation, Qdrant upsertions) in `backend/main.py`.
- [X] T019 Add input validation for incoming data (e.g., URL formats) to ingestion functions in `backend/main.py`.
- [X] T020 Review and add inline comments/docstrings to `backend/main.py` for maintainability.
- [X] T021 Validate the implemented pipeline against the `quickstart.md` steps (manual check).

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
-   **User Stories (Phase 3+)**: All depend on Foundational phase completion.
    *   User stories can then proceed in parallel (if staffed).
    *   Or sequentially in priority order (P1 → P2).
-   **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
-   **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Enhances US1's content extraction.

### Within Each User Story

-   Models before services.
-   Core implementation before integration.
-   Story complete before moving to next priority.

### Parallel Opportunities

-   **Phase 1**: T002, T003 can be done in parallel.
-   **Phase 3**: T010, T011, T012 can be done in parallel once `TextChunk` and clients are initialized.

---

## Parallel Example: User Story 1

```bash
# Once Foundational Phase is complete, T010, T011, T012 can run in parallel:
# (These tasks involve external API calls and Playwright, which can be setup concurrently)
Task: "Implement primary content extraction function using Playwright (e.g., `extract_content_playwright`) in `backend/main.py`"
Task: "Implement embedding generation function using Cohere API (e.g., `generate_embedding_cohere`) in `backend/main.py`"
Task: "Implement Qdrant storage function (e.g., `store_chunks_in_qdrant`) to upsert vectors and payloads in `backend/main.py`"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test User Story 1 independently
5.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Add User Story 2 → Test independently → Deploy/Demo
4.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together
2.  Once Foundational is done:
    *   Developer A: User Story 1
    *   Developer B: User Story 2
3.  Stories complete and integrate independently

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
