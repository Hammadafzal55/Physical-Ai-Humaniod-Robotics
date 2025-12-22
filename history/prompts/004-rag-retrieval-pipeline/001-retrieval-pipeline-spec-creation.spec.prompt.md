---
id: 001
title: Retrieval Pipeline Spec Creation
stage: spec
date: 2025-12-22
surface: agent
model: gemini-1.5-flash-latest
feature: 004-rag-retrieval-pipeline
branch: main
user: User
command: /sp.specify SPEC NAME: Spec 4: Retrieval Pipeline (Vector Search & Context Assembly) CONTEXT: - Project: AI-native textbook + RAG chatbot system. - Spec 3 (RAG Pipeline Setup & Embedding Storage) is **completed**. - In Spec 3: - Textbook content was discovered via sitemap → URLs - Pages were fetched using **mandatory Playwright-based scraping** - Content was cleaned, chunked, embedded using **Cohere `embed-english-v3.0`** - Embeddings were stored in **Qdrant** - All ingestion logic lives in `backend/main.py` - All work was done strictly on the **main branch** This spec builds **directly on top of Spec 3 outputs** and MUST NOT re-ingest, re-scrape, or re-embed content. OBJECTIVE: Build a reliable retrieval pipeline that: - Queries Qdrant using vector similarity - Retrieves the most relevant textbook chunks - Assembles clean, ordered context - Prepares retrieved data for downstream agent usage (Spec 5) SCOPE: Includes: - Connecting to the existing Qdrant collection created in Spec 3 - Accepting a user query as input - Generating a query embedding using **Cohere `embed-english-v3.0`** - Performing similarity search against stored vectors - Retrieving top-K relevant chunks - Returning structured, citation-ready context (text + metadata) ARCHITECTURE & FILE RULES: - Backend code lives in `/backend` - Retrieval logic MUST be implemented in: - `retrieval.py` - `main.py` MUST NOT be modified unless absolutely required - Retrieval code must reuse: - Qdrant collection - Embedding dimensions - Metadata schema defined in Spec 3 - No duplication of embedding or ingestion logic RETRIEVAL REQUIREMENTS: - Retrieval must be deterministic and testable - Results must be ranked by relevance score - Returned chunks must include: - Chunk text - Source URL - Module / chapter metadata - Context must be safe for direct consumption by an agent - Retrieval must fail gracefully if no relevant data is found QUALITY CONSTRAINTS: - Retrieval must ONLY return textbook-derived data - No hallucinated or external content - Chunk ordering and cleanliness must be preserved - Performance must be suitable for low-end devices GIT & WORKFLOW RULES: - All work MUST remain on the **main branch** - ❌ No new branches - ❌ No branch switching - ❌ No rewriting Spec 3 code unless strictly required DEFINITION OF DONE: - A working retrieval pipeline implemented in `retrieval.py` - Queries correctly return relevant textbook chunks from Qdrant - Output is structured and ready for agent consumption - Spec 4 fully aligned with decisions and constraints from Spec 3
labels: ["rag-retrieval", "spec-creation"]
links:
  spec: specs/004-rag-retrieval-pipeline/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-retrieval-pipeline/spec.md
  - specs/004-rag-retrieval-pipeline/checklists/requirements.md
tests:
  - (none)
---

## Prompt

SPEC NAME: Spec 4: Retrieval Pipeline (Vector Search & Context Assembly) CONTEXT: - Project: AI-native textbook + RAG chatbot system. - Spec 3 (RAG Pipeline Setup & Embedding Storage) is **completed**. - In Spec 3: - Textbook content was discovered via sitemap → URLs - Pages were fetched using **mandatory Playwright-based scraping** - Content was cleaned, chunked, embedded using **Cohere `embed-english-v3.0`** - Embeddings were stored in **Qdrant** - All ingestion logic lives in `backend/main.py` - All work was done strictly on the **main branch** This spec builds **directly on top of Spec 3 outputs** and MUST NOT re-ingest, re-scrape, or re-embed content. OBJECTIVE: Build a reliable retrieval pipeline that: - Queries Qdrant using vector similarity - Retrieves the most relevant textbook chunks - Assembles clean, ordered context - Prepares retrieved data for downstream agent usage (Spec 5) SCOPE: Includes: - Connecting to the existing Qdrant collection created in Spec 3 - Accepting a user query as input - Generating a query embedding using **Cohere `embed-english-v3.0`** - Performing similarity search against stored vectors - Retrieving top-K relevant chunks - Returning structured, citation-ready context (text + metadata) ARCHITECTURE & FILE RULES: - Backend code lives in `/backend` - Retrieval logic MUST be implemented in: - `retrieval.py` - `main.py` MUST NOT be modified unless absolutely required - Retrieval code must reuse: - Qdrant collection - Embedding dimensions - Metadata schema defined in Spec 3 - No duplication of embedding or ingestion logic RETRIEVAL REQUIREMENTS: - Retrieval must be deterministic and testable - Results must be ranked by relevance score - Returned chunks must include: - Chunk text - Source URL - Module / chapter metadata - Context must be safe for direct consumption by an agent - Retrieval must fail gracefully if no relevant data is found QUALITY CONSTRAINTS: - Retrieval must ONLY return textbook-derived data - No hallucinated or external content - Chunk ordering and cleanliness must be preserved - Performance must be suitable for low-end devices GIT & WORKFLOW RULES: - All work MUST remain on the **main branch** - ❌ No new branches - ❌ No branch switching - ❌ No rewriting Spec 3 code unless strictly required DEFINITION OF DONE: - A working retrieval pipeline implemented in `retrieval.py` - Queries correctly return relevant textbook chunks from Qdrant - Output is structured and ready for agent consumption - Spec 4 fully aligned with decisions and constraints from Spec 3

## Response snapshot

The specification for "Retrieval Pipeline (Vector Search & Context Assembly)" (`specs/004-rag-retrieval-pipeline/spec.md`) has been created and validated. All items in the quality checklist (`specs/004-rag-retrieval-pipeline/checklists/requirements.md`) have passed.

## Outcome

- ✅ Impact: Specification for Retrieval Pipeline (Vector Search & Context Assembly) created and validated.
- 🧪 Tests: No automated tests were run as part of spec creation.
- 📁 Files: Created `specs/004-rag-retrieval-pipeline/spec.md` and `specs/004-rag-retrieval-pipeline/checklists/requirements.md`.
- 🔁 Next prompts: Next step is to create a plan using `/sp.plan`.
- 🧠 Reflection: Successfully created a feature specification and updated the checklist. Adhered to single-branch development constraint.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): None.
- Prompt variant (if applicable): None.
- Next experiment (smallest change to try): None.
