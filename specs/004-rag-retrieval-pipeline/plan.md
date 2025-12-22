# Implementation Plan: Retrieval Pipeline (Vector Search & Context Assembly)

**Branch**: `main` | **Date**: 2025-12-22 | **Spec**: specs/004-rag-retrieval-pipeline/spec.md
**Input**: Feature specification from `specs/004-rag-retrieval-pipeline/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation of a reliable retrieval pipeline for the AI-native textbook + RAG chatbot system. It will query the existing Qdrant vector database using vector similarity, retrieve the most relevant textbook chunks, assemble them into clean, ordered context, and prepare this data for downstream agent consumption. The logic will reside in `backend/retrieval.py`, building directly on outputs from Spec 3 (RAG Pipeline Setup & Embedding Storage) and strictly adhering to the reuse of existing Qdrant collection, embedding dimensions, and metadata schema.

## Technical Context

**Language/Version**: Python 3.11 (inherited from Spec 3, uv package environment)
**Primary Dependencies**: Cohere Python SDK, Qdrant Python Client, FastAPI (for integration if needed, but core retrieval logic will be standalone in `retrieval.py`)
**Storage**: Qdrant Vector Database (existing collection from Spec 3)
**Testing**: pytest (common for Python projects, will be confirmed during task breakdown)
**Target Platform**: Linux server (standard for backend services)
**Project Type**: Backend service component (integrated with FastAPI app, if /retrieve endpoint is separate from `main.py`)
**Performance Goals**: Retrieval latency for 95% of queries under 5 seconds, even on low-end devices.
**Constraints**:
    *   Retrieval logic MUST be implemented in `backend/retrieval.py`.
    *   `main.py` MUST NOT be modified unless absolutely required.
    *   Must reuse Qdrant collection, embedding dimensions, metadata schema from Spec 3.
    *   No duplication of embedding or ingestion logic.
    *   Retrieval must query **only Qdrant**.
    *   No scraping, no ingestion, no re-embedding of textbook content.
    *   All Git work MUST remain on the **main branch**.
**Scale/Scope**: Retrieval pipeline for an AI-native textbook.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**7.1. SINGLE-BRANCH DEVELOPMENT ONLY**: PASSED. All development is confirmed to be on the `main` branch as per project constitution and planning rules.

## Project Structure

### Documentation (this feature)

```text
specs/004-rag-retrieval-pipeline/
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
├── retrieval.py # All core retrieval logic will be implemented here.
└── main.py      # Existing ingestion logic and FastAPI endpoint. Must not be modified unless absolutely required.
└── uv.lock
└── pyproject.toml
└── .python-version
└── README.md
└── .venv
frontend/
└── ... (existing frontend files)
```

**Structure Decision**: A new file `backend/retrieval.py` will host all core retrieval logic, ensuring `backend/main.py` remains dedicated to ingestion and minimal API scaffolding. This aligns with the architectural principle of modularity and separation of concerns.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| | | |
