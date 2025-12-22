# Implementation Plan: RAG Pipeline Setup & Embedding Storage

**Branch**: `main` | **Date**: 2025-12-21 | **Spec**: specs/003-rag-pipeline-setup/spec.md
**Input**: Feature specification from `specs/003-rag-pipeline-setup/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan details the implementation of a RAG ingestion pipeline to extract textbook content from deployed frontend URLs, generate embeddings using Cohere `embed-english-v3.0`, and store them in Qdrant. Content extraction will primarily rely on Playwright for robust and reliable scraping. The backend logic will reside in `backend/main.py` and adhere to the `uv package` structure, ensuring modularity, maintainability, and readiness for future agent integration.

## Technical Context

**Language/Version**: Python 3.11 (as implied by `uv package` and `backend/main.py`)
**Primary Dependencies**: Playwright, Cohere Python SDK, Qdrant Python SDK, FastAPI (implied by `backend` folder structure for a web service).
**Storage**: Qdrant Vector Database
**Testing**: pytest (common for Python projects, will be confirmed during task breakdown)
**Target Platform**: Linux server (standard for backend services)
**Project Type**: Backend service (integrated with an existing frontend)
**Performance Goals**:
    *   Content extraction for a single textbook page SHALL complete within 30 seconds.
    *   Embedding generation for a typical text chunk (e.g., 500 tokens) SHALL complete within 5 seconds.
    *   Storage of an embedding and its metadata in Qdrant SHALL complete within 100 milliseconds.
**Constraints**:
    *   All backend logic must reside in `backend/main.py`.
    *   Existing `uv package` structure must be used, with all dependencies already installed.
    *   Playwright is mandatory for content fetching and extraction.
    *   No new branch creation; all work on `main`.
**Scale/Scope**: Ingestion pipeline for an AI-native textbook (initial scope, scalable for future content).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**7.1. SINGLE-BRANCH DEVELOPMENT ONLY**: PASSED. All development is confirmed to be on the `main` branch as per project constitution and planning rules.

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-pipeline-setup/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py # All Phase 2 pipeline code will be integrated here, potentially extending existing functionality.
└── uv.lock
└── pyproject.toml
└── .python-version
└── README.md
└── .venv
frontend/
└── ... (existing frontend files)
```

**Structure Decision**: The existing `backend/` folder structure will be utilized, with `main.py` serving as the central file for all RAG pipeline logic. This aligns with the `uv package` setup and the requirement for a single modular file.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| | | |
