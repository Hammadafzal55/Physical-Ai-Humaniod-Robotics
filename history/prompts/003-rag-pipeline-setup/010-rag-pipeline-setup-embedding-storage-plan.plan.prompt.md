---
id: 002
title: RAG Pipeline Setup Embedding Storage Plan
stage: plan
date: 2025-12-21
surface: agent
model: gemini-1.5-flash-latest
feature: 003-rag-pipeline-setup
branch: main
user: User
command: /sp.plan
labels: ["rag-pipeline", "plan-creation"]
links:
  spec: specs/003-rag-pipeline-setup/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-rag-pipeline-setup/plan.md
  - specs/003-rag-pipeline-setup/research.md
  - specs/003-rag-pipeline-setup/data-model.md
  - specs/003-rag-pipeline-setup/quickstart.md
  - specs/003-rag-pipeline-setup/contracts/retrieval_contract.json
  - GEMINI.md
tests:
  - (none)
---

## Prompt

/sp.plan

SPEC NAME:
Spec 3: RAG Pipeline Setup & Embedding Storage

OBJECTIVE:
Plan the RAG ingestion pipeline to extract textbook content, generate embeddings using Cohere `embed-english-v3.0`, and store them in Qdrant using a reliable scraping strategy.

PLANNING RULES:

1. All backend logic lives in `backend/main.py`.
   - `main.py` may already exist.
   - Modify `main.py` **only when required**, e.g., to integrate or extend Playwright-based scraping.
2. Use **uv package** structure; backend is already initialized and all dependencies are installed.
3. Use **deployed frontend URLs** as the source of textbook content.
4. Extract only real chapter/page content; the sitemap is strictly for discovery only.
5. Generate embeddings using **Cohere `embed-english-v3.0`**.
6. Store embeddings in **Qdrant** with proper metadata (module, chapter, URL).
7. **Playwright is mandatory** for content fetching and extraction.
   - Reason: standard web scraping is repeatedly blocked by Vercel.
   - Playwright must be the primary and reliable mechanism for page rendering and content extraction.
8. Keep the pipeline modular, maintainable, and ready for future retrieval and agent integration.
9. Ensure the pipeline is safe, testable, and reusable.
10. Preserve deployed URLs access for validation and testing.
11. Follow the mental model strictly:
    Sitemap → URLs → Pages → Text → Chunks → Embeddings → Vector DB
12. **All work must be done strictly on the `main` branch.**
    - ❌ No feature/dev/experiment branches
    - ❌ No branch switching

## Response snapshot

The implementation plan for "RAG Pipeline Setup & Embedding Storage" (`specs/003-rag-pipeline-setup/plan.md`) has been created, and related design artifacts have been generated. The agent's knowledge base has also been updated with the project's key technologies.

## Outcome

- ✅ Impact: Implementation plan and design artifacts for RAG pipeline created. Agent context updated.
- 🧪 Tests: No automated tests were run during planning.
- 📁 Files: Created `specs/003-rag-pipeline-setup/plan.md`, `specs/003-rag-pipeline-setup/research.md`, `specs/003-rag-pipeline-setup/data-model.md`, `specs/003-rag-pipeline-setup/quickstart.md`, `specs/003-rag-pipeline-setup/contracts/retrieval_contract.json`. Updated `GEMINI.md`.
- 🔁 Next prompts: Next step is to break the plan into tasks using `/sp.tasks`.
- 🧠 Reflection: Successfully generated an implementation plan, performed necessary research, defined data model and API contracts, and updated the agent's knowledge base, adhering to all project constraints including single-branch development.

## Evaluation notes (flywheel)

- Failure modes observed: The `setup-plan.ps1` script incorrectly identified `specs/main/` as the feature directory, leading to an incorrect `plan.md` path in the script output and incorrect parsing by `update-agent-context.ps1`. This required manual correction of paths and manual update of `GEMINI.md`.
- Graders run and results (PASS/FAIL): None.
- Prompt variant (if applicable): None.
- Next experiment (smallest change to try): Review and update `setup-plan.ps1` and `update-agent-context.ps1` to correctly infer the feature directory from the context (e.g., from the `SPEC NAME` argument or by searching `specs/` for the latest feature spec).
