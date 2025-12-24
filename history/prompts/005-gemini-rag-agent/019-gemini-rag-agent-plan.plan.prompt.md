---
id: 019
title: Gemini RAG Agent Plan
stage: plan
date: 2025-12-22
surface: agent
model: gemini-1.5-flash-latest
feature: 005-gemini-rag-agent
branch: main
user: User
command: /sp.plan SPEC NAME: Spec 5: Agent Development (Gemini-Powered RAG Agent) CONTEXT: - Project: AI-native textbook + RAG chatbot system. - Spec 3 (Ingestion & Embeddings) is complete. - Spec 4 (Retrieval Pipeline) is complete. - This spec builds strictly on top of Spec 3 and Spec 4. OBJECTIVE: Build a **Gemini-powered chatbot agent** that: - Handles simple greetings directly - Uses retrieval tools for textbook-related questions - Answers strictly from textbook content - Allows **limited creativity in explanation**, without changing meaning SCOPE: Includes: - Gemini-based agent implementation - Retrieval access via `@function_tool` - Controlled creative explanation - FastAPI endpoints for chatbot interaction ARCHITECTURE & FILE RULES: - Backend code lives in `/backend` - Agent logic MUST be implemented in: - `agent.py` - API routes MUST be implemented in: - `api.py` - Retrieval logic remains in: - `retrieval.py` - `main.py` MUST NOT be modified - Agent must access textbook data **only via `@function_tool`** AGENT BEHAVIOR RULES: 1. **Greeting Behavior (Direct Response)**: - If user says: - "hello" - "hi" - "who are you?" - Agent may respond directly with a short predefined message. 2. **Textbook Question Behavior (Strict RAG)**: - For any textbook-related question: - Agent MUST call retrieval using `@function_tool` - Agent MUST answer **only using tool-returned context** - No external knowledge allowed 3. **Controlled Creativity Rule**: - Agent MAY: - Rephrase retrieved text - Improve clarity and readability - Add explanatory flow - Agent MUST NOT: - Add new facts - Change meaning - Introduce external concepts 4. **No-Data Behavior**: - If retrieval returns no relevant content: - Agent must clearly say the answer is not available in the textbook. TOOLING REQUIREMENTS: - Retrieval exposed as a function tool - Agent cannot access Qdrant or database directly - Tool output must include: - Chunk text - Source URL - Module / chapter metadata FASTAPI REQUIREMENTS: - Chat endpoint implemented in `api.py` - `api.py` should: - Accept user message - Pass it to the agent - Return agent response - Designed so deployment requires only: - `api.py` - `agent.py` QUALITY CONSTRAINTS: - Responses must be clear, educational, and friendly - Creativity allowed **only in expression** - Zero hallucinations - No external knowledge leakage GIT & WORKFLOW RULES: - All work MUST remain on the **main branch** - ❌ No new branches - ❌ No branch switching
labels: ["gemini-rag-agent", "plan-creation"]
links:
  spec: specs/005-gemini-rag-agent/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-gemini-rag-agent/plan.md
  - specs/005-gemini-rag-agent/research.md
  - specs/005-gemini-rag-agent/data-model.md
  - specs/005-gemini-rag-agent/quickstart.md
  - specs/005-gemini-rag-agent/contracts/chat_api_contract.py
  - GEMINI.md
tests:
  - (none)
---

## Prompt

/sp.plan

SPEC NAME:
Phase 2 — Spec 3: Agent Development (Gemini-Powered RAG Agent)

OBJECTIVE:
Plan a Gemini-powered RAG agent that handles greetings directly, answers textbook-related questions strictly using retrieved data via `@function_tool`, allows limited creativity in explanation.

PLANNING RULES:

1. This spec is a direct continuation of:
- Spec 4 (Ingestion & Embeddings)
- Spec 5 (Retrieval Pipeline)

2. Agent logic MUST live in:
   - `backend/agent.py`

3. API routing MUST live in:
   - `backend/api.py`

4. Retrieval access:
   - Agent MUST access textbook data only via `@function_tool`.
   - Retrieval logic remains in `retrieval.py`.
   - Direct database or Qdrant access from agent is NOT allowed.

5. Agent behavior rules:
   - Simple greetings (e.g., "hello", "hi", "who are you?") handled directly without retrieval.
   - All textbook-related questions MUST trigger retrieval.
   - Answers MUST be based only on tool-returned context.

6. Controlled creativity rule:
   - Creativity allowed only in explanation style and clarity.
   - Facts, concepts, and meaning MUST remain unchanged.
   - No external knowledge or new information allowed.

7. Prompt control:
   - Agent must use a fixed, non-overridable system instruction.
   - User input must be injected as a variable and must not override grounding rules.
   - Retrieved chunks must be provided as controlled context.

8. No-data handling:
   - If retrieval returns no relevant content, agent must clearly state that the answer is not available in the textbook.

9. Performance & safety:
   - Responses must be suitable for real-time chat usage.
   - Output must be safe for direct rendering in the chatbot UI.
   - No hallucinations or external leakage.

10. Architecture discipline:
    - `main.py` and `retrieval.py` MUST NOT be modified unless strictly required.
    - No duplication of retrieval or embedding logic.

11. Git & workflow rules:
    - All work MUST remain on the **main branch**.
    - ❌ No new branches
    - ❌ No branch switching

## Response snapshot

The implementation plan for "Agent Development (Gemini-Powered RAG Agent)" (`specs/005-gemini-rag-agent/plan.md`) has been created, and related design artifacts have been generated. The agent's knowledge base (`GEMINI.md`) has been updated.

## Outcome

- ✅ Impact: Implementation plan and design artifacts for Gemini RAG Agent created. Agent context updated.
- 🧪 Tests: No automated tests were run during planning.
- 📁 Files: Created `specs/005-gemini-rag-agent/plan.md`, `specs/005-gemini-rag-agent/research.md`, `specs/005-gemini-rag-agent/data-model.md`, `specs/005-gemini-rag-agent/quickstart.md`, `specs/005-gemini-rag-agent/contracts/chat_api_contract.py`. Updated `GEMINI.md`.
- 🔁 Next prompts: Next step is to break the plan into tasks using `/sp.tasks`.
- 🧠 Reflection: Successfully generated an implementation plan, performed necessary research, defined data model and API contracts, and updated the agent's knowledge base, adhering to all project constraints including single-branch development.

## Evaluation notes (flywheel)

- Failure modes observed: The `setup-plan.ps1` script incorrectly identified `specs/main/` as the feature directory.
- Graders run and results (PASS/FAIL): None.
- Prompt variant (if applicable): None.
- Next experiment (smallest change to try): Improve `setup-plan.ps1` to correctly identify the feature directory from the context (e.g., from the `SPEC NAME` argument or by searching `specs/` for the latest feature spec).
