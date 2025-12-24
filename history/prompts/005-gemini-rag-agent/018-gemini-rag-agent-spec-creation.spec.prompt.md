---
id: 018
title: Gemini RAG Agent Spec Creation
stage: spec
date: 2025-12-22
surface: agent
model: gemini-1.5-flash-latest
feature: 005-gemini-rag-agent
branch: main
user: User
command: /sp.specify SPEC NAME: Spec 5: Agent Development (Gemini-Powered RAG Agent) CONTEXT: - Project: AI-native textbook + RAG chatbot system. - Spec 3 (Ingestion & Embeddings) is complete. - Spec 4 (Retrieval Pipeline) is complete. - This spec builds strictly on top of Spec 3 and Spec 4. OBJECTIVE: Build a **Gemini-powered chatbot agent** that: - Handles simple greetings directly - Uses retrieval tools for textbook-related questions - Answers strictly from textbook content - Allows **limited creativity in explanation**, without changing meaning SCOPE: Includes: - Gemini-based agent implementation - Retrieval access via `@function_tool` - Controlled creative explanation - FastAPI endpoints for chatbot interaction ARCHITECTURE & FILE RULES: - Backend code lives in `/backend` - Agent logic MUST be implemented in: - `agent.py` - API routes MUST be implemented in: - `api.py` - Retrieval logic remains in: - `retrieval.py` - `main.py` MUST NOT be modified - Agent must access textbook data **only via `@function_tool`** AGENT BEHAVIOR RULES: 1. **Greeting Behavior (Direct Response)**: - If user says: - "hello" - "hi" - "who are you?" - Agent may respond directly with a short predefined message. 2. **Textbook Question Behavior (Strict RAG)**: - For any textbook-related question: - Agent MUST call retrieval using `@function_tool` - Agent MUST answer **only using tool-returned context** - No external knowledge allowed 3. **Controlled Creativity Rule**: - Agent MAY: - Rephrase retrieved text - Improve clarity and readability - Add explanatory flow - Agent MUST NOT: - Add new facts - Change meaning - Introduce external concepts 4. **No-Data Behavior**: - If retrieval returns no relevant content: - Agent must clearly say the answer is not available in the textbook. TOOLING REQUIREMENTS: - Retrieval exposed as a function tool - Agent cannot access Qdrant or database directly - Tool output must include: - Chunk text - Source URL - Module / chapter metadata FASTAPI REQUIREMENTS: - Chat endpoint implemented in `api.py` - `api.py` should: - Accept user message - Pass it to the agent - Return agent response - Designed so deployment requires only: - `api.py` - `agent.py` QUALITY CONSTRAINTS: - Responses must be clear, educational, and friendly - Creativity allowed **only in expression** - Zero hallucinations - No external knowledge leakage GIT & WORKFLOW RULES: - All work MUST remain on the **main branch** - ❌ No new branches - ❌ No branch switching
labels: ["gemini-rag-agent", "spec-creation"]
links:
  spec: specs/005-gemini-rag-agent/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-gemini-rag-agent/spec.md
  - specs/005-gemini-rag-agent/checklists/requirements.md
tests:
  - (none)
---

## Prompt

SPEC NAME: Spec 5: Agent Development (Gemini-Powered RAG Agent) CONTEXT: - Project: AI-native textbook + RAG chatbot system. - Spec 3 (Ingestion & Embeddings) is complete. - Spec 4 (Retrieval Pipeline) is complete. - This spec builds strictly on top of Spec 3 and Spec 4. OBJECTIVE: Build a **Gemini-powered chatbot agent** that: - Handles simple greetings directly - Uses retrieval tools for textbook-related questions - Answers strictly from textbook content - Allows **limited creativity in explanation**, without changing meaning SCOPE: Includes: - Gemini-based agent implementation - Retrieval access via `@function_tool` - Controlled creative explanation - FastAPI endpoints for chatbot interaction ARCHITECTURE & FILE RULES: - Backend code lives in `/backend` - Agent logic MUST be implemented in: - `agent.py` - API routes MUST be implemented in: - `api.py` - Retrieval logic remains in: - `retrieval.py` - `main.py` MUST NOT be modified - Agent must access textbook data **only via `@function_tool`** AGENT BEHAVIOR RULES: 1. **Greeting Behavior (Direct Response)**: - If user says: - "hello" - "hi" - "who are you?" - Agent may respond directly with a short predefined message. 2. **Textbook Question Behavior (Strict RAG)**: - For any textbook-related question: - Agent MUST call retrieval using `@function_tool` - Agent MUST answer **only using tool-returned context** - No external knowledge allowed 3. **Controlled Creativity Rule**: - Agent MAY: - Rephrase retrieved text - Improve clarity and readability - Add explanatory flow - Agent MUST NOT: - Add new facts - Change meaning - Introduce external concepts 4. **No-Data Behavior**: - If retrieval returns no relevant content: - Agent must clearly say the answer is not available in the textbook. TOOLING REQUIREMENTS: - Retrieval exposed as a function tool - Agent cannot access Qdrant or database directly - Tool output must include: - Chunk text - Source URL - Module / chapter metadata FASTAPI REQUIREMENTS: - Chat endpoint implemented in `api.py` - `api.py` should: - Accept user message - Pass it to the agent - Return agent response - Designed so deployment requires only: - `api.py` - `agent.py` QUALITY CONSTRAINTS: - Responses must be clear, educational, and friendly - Creativity allowed **only in expression** - Zero hallucinations - No external knowledge leakage GIT & WORKFLOW RULES: - All work MUST remain on the **main branch** - ❌ No new branches - ❌ No branch switching

## Response snapshot

The specification for "Agent Development (Gemini-Powered RAG Agent)" (`specs/005-gemini-rag-agent/spec.md`) has been created and validated. All items in the quality checklist (`specs/005-gemini-rag-agent/checklists/requirements.md`) have passed.

## Outcome

- ✅ Impact: Specification for Agent Development (Gemini-Powered RAG Agent) created and validated.
- 🧪 Tests: No automated tests were run as part of spec creation.
- 📁 Files: Created `specs/005-gemini-rag-agent/spec.md` and `specs/005-gemini-rag-agent/checklists/requirements.md`.
- 🔁 Next prompts: Next step is to create a plan using `/sp.plan`.
- 🧠 Reflection: Successfully created a feature specification and updated the checklist. Adhered to single-branch development constraint.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): None.
- Prompt variant (if applicable): None.
- Next experiment (smallest change to try): None.
