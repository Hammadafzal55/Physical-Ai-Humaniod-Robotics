---
id: 015
title: Retrieval Pipeline Plan
stage: plan
date: 2025-12-22
surface: agent
model: gemini-1.5-flash-latest
feature: 004-rag-retrieval-pipeline
branch: main
user: User
command: /sp.plan SPEC NAME: Spec 4: Retrieval Pipeline (Vector Search & Context Assembly) CONTEXT: - Project: AI-native textbook + RAG chatbot system. - Spec 3 (RAG Pipeline Setup & Embedding Storage) is **completed**. - In Spec 3: - Textbook content was discovered via sitemap → URLs - Pages were fetched using **mandatory Playwright-based scraping** - Content was cleaned, chunked, embedded using **Cohere `embed-english-v3.0`** - Embeddings were stored in **Qdrant** - All ingestion logic lives in `backend/main.py` - All work was done strictly on the **main branch** This spec builds **directly on top of Spec 3 outputs** and MUST NOT re-ingest, re-scrape, or re-embed content. OBJECTIVE: Build a reliable retrieval pipeline that: - Queries Qdrant using vector similarity - Retrieves the most relevant textbook chunks - Assembles clean, ordered context - Prepares retrieved data for downstream agent usage (Spec 5) SCOPE: Includes: - Connecting to the existing Qdrant collection created in Spec 3 - Accepting a user query as input - Generating a query embedding using **Cohere `embed-english-v3.0`** - Performing similarity search against stored vectors - Retrieving top-K relevant chunks - Returning structured, citation-ready context (text + metadata) ARCHITECTURE & FILE RULES: - Backend code lives in `/backend` - Retrieval logic MUST be implemented in: - `retrieval.py` - `main.py` MUST NOT be modified unless absolutely required - Retrieval code must reuse: - Qdrant collection - Embedding dimensions - Metadata schema defined in Spec 3 - No duplication of embedding or ingestion logic RETRIEVAL REQUIREMENTS: - Retrieval must be deterministic and testable - Results must be ranked by relevance score - Returned chunks must include: - Chunk text - Source URL - Module / chapter metadata - Context must be safe for direct consumption by an agent - Retrieval must fail gracefully if no relevant data is found QUALITY CONSTRAINTS: - Retrieval must ONLY return textbook-derived data - No hallucinated or external content - Chunk ordering and cleanliness must be preserved - Performance must be suitable for low-end devices GIT & WORKFLOW RULES: - All work MUST remain on the **main branch** - ❌ No new branches - ❌ No branch switching - ❌ No rewriting Spec 3 code unless strictly required DEFINITION OF DONE: - A working retrieval pipeline implemented in `retrieval.py` - Queries correctly return relevant textbook chunks from Qdrant - Output is structured and ready for agent consumption - Spec 4 fully aligned with decisions and constraints from Spec 3
labels: ["rag-retrieval", "plan-creation"]
links:
  spec: specs/004-rag-retrieval-pipeline/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-retrieval-pipeline/plan.md
  - specs/004-rag-retrieval-pipeline/research.md
  - specs/004-rag-retrieval-pipeline/data-model.md
  - specs/004-rag-retrieval-pipeline/quickstart.md
  - specs/004-rag-retrieval-pipeline/contracts/retrieval_service_contract.py
tests:
  - (none)
---

## Prompt

/sp.plan

SPEC NAME:
Spec 4: Retrieval Pipeline (Vector Search & Context Assembly)

OBJECTIVE:
Plan the retrieval pipeline to query the existing Qdrant vector database, retrieve the most relevant textbook chunks using vector similarity, and prepare clean, structured context for agent consumption.

PLANNING RULES:

1. All retrieval logic lives in `backend/retrieval.py`.
2. Backend uses **uv package** structure; project is already initialized with dependencies installed.
3. Reuse the **same Qdrant collection** created in Spec 3.
4. Generate query embeddings using **Cohere `embed-english-v3.0`**.
   - Embedding dimensions and preprocessing must exactly match Spec 1.
5. Retrieval must query **only Qdrant**.
   - No scraping
   - No ingestion
   - No re-embedding of textbook content
6. Retrieved results must include:
   - Chunk text
   - Source URL
   - Module and chapter metadata
7. Results must be ranked by similarity score and relevance.
8. Retrieval output must be:
   - Clean
   - Deterministic
   - Ready for direct agent usage
9. Retrieval must return **only textbook-derived content**.
   - No external or hallucinated data.
10. Retrieval must handle empty or low-relevance results safely.

11. All Git work must remain on the **main branch**.
    - ❌ No new branches
    - ❌ No branch switching

## Response snapshot

The implementation plan for "Retrieval Pipeline (Vector Search & Context Assembly)" (`specs/004-rag-retrieval-pipeline/plan.md`) has been created, and related design artifacts have been generated.

## Outcome

- ✅ Impact: Implementation plan and design artifacts for Retrieval Pipeline created.
- 🧪 Tests: No automated tests were run during planning.
- 📁 Files: Created `specs/004-rag-retrieval-pipeline/plan.md`, `specs/004-rag-retrieval-pipeline/research.md`, `specs/004-rag-retrieval-pipeline/data-model.md`, `specs/004-rag-retrieval-pipeline/quickstart.md`, `specs/004-rag-retrieval-pipeline/contracts/retrieval_service_contract.py`.
- 🔁 Next prompts: Next step is to break the plan into tasks using `/sp.tasks`.
- 🧠 Reflection: Successfully generated an implementation plan, performed necessary research, defined data model and API contracts, adhering to all project constraints including single-branch development.

## Evaluation notes (flywheel)

- Failure modes observed: The `setup-plan.ps1` script incorrectly identified `specs/main/` as the feature directory.
- Graders run and results (PASS/FAIL): None.
- Prompt variant (if applicable): None.
- Next experiment (smallest change to try): Improve `setup-plan.ps1` to correctly identify the feature directory from the context (e.g., from the `SPEC NAME` argument or by searching `specs/` for the latest feature spec).
