# Implementation Plan: Agent Development (Gemini-Powered RAG Agent)

**Branch**: `main` | **Date**: 2025-12-22 | **Spec**: specs/005-gemini-rag-agent/spec.md
**Input**: Feature specification from `specs/005-gemini-rag-agent/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation of a Gemini-powered RAG chatbot agent. The agent will handle simple greetings directly, use a retrieval tool (from Spec 4) for textbook-related questions, and answer strictly from the retrieved textbook content. It will allow limited creativity in explanation style while maintaining factual accuracy. The agent logic will reside in `backend/agent.py`, and FastAPI routes for chatbot interaction will be implemented in `backend/api.py`.

## Technical Context

**Language/Version**: Python 3.11 (inherited from previous specs, uv package environment)
**Primary Dependencies**: Google Gemini API, `backend/retrieval.py` (for retrieval tool), FastAPI (for API routes)
**Storage**: N/A (agent is stateless, relies on Qdrant via retrieval tool)
**Testing**: pytest (common for Python projects, will be confirmed during task breakdown)
**Target Platform**: Linux server (standard for backend services)
**Project Type**: Backend service component (Gemini-powered RAG agent with FastAPI interface)
**Performance Goals**: Responses suitable for real-time chat usage; FastAPI `/chat` endpoint response time (p95) under 5 seconds.
**Constraints**:
    *   Agent logic MUST be implemented in `backend/agent.py`.
    *   API routes MUST be implemented in `backend/api.py`.
    *   `retrieval.py` remains unchanged.
    *   `main.py` MUST NOT be modified.
    *   Agent must access textbook data **only via `@function_tool`**.
    *   All Git work MUST remain on the **main branch**.
    *   Agent must use a fixed, non-overridable system instruction.
    *   User input must be injected as a variable and must not override grounding rules.
    *   Retrieved chunks must be provided as controlled context.
**Scale/Scope**: Chatbot agent for an AI-native textbook.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**7.1. SINGLE-BRANCH DEVELOPMENT ONLY**: PASSED. All development is confirmed to be on the `main` branch as per project constitution and planning rules.

## Project Structure

### Documentation (this feature)

```text
specs/005-gemini-rag-agent/
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
├── agent.py     # All core agent logic will be implemented here.
├── api.py       # All FastAPI chat routes will be implemented here.
├── retrieval.py # Existing retrieval logic (unchanged).
└── main.py      # Existing ingestion logic and FastAPI endpoint (unchanged).
└── uv.lock
└── pyproject.toml
└── .python-version
└── README.md
└── .venv
frontend/
└── ... (existing frontend files)
```

**Structure Decision**: A new file `backend/agent.py` will host all core Gemini agent logic. `backend/api.py` will contain FastAPI routes specifically for chatbot interaction. `retrieval.py` and `main.py` will remain untouched, adhering to the architectural constraints.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| | | |
