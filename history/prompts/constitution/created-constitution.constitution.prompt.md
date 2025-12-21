---
id: 1
title: Update Project Constitution with Initial Details
stage: constitution
date: 2025-12-20
surface: agent
model: gemini-1.5-pro
feature: none
branch: main
user: User
command: /sp.constitution
labels: ["constitution", "initial-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: []
---

## Prompt

/sp.constitution /sp.constitution

PROJECT: AI-Native Textbook + RAG Chatbot for Physical AI & Humanoid Robotics

CORE SCOPE
This project is divided into TWO phases for building:
1) Textbook generation with UI and chat interface
2) RAG-based chatbot built on top of the textbook

The project focuses only on:
- Building a complete AI-native textbook
- Building a RAG chatbot grounded strictly in that textbook

MISSION
Build a modern, detailed, AI-generated textbook for Physical AI & Humanoid Robotics and then layer a retrieval-augmented chatbot on top of it.
The chatbot must answer questions using only the textbook content.

TECH STACK

FRONTEND
- Docusaurus (documentation-based textbook)
- React (UI components)
- TypeScript
- CSS Modules / Tailwind CSS (lightweight styling)

BACKEND
- Python 3.11+
- FastAPI (API layer)
- uv (Python package & project manager)

AI & ML
- Embedding Model: Cohere `embed-english-v3.0`
- Agent Model: Gemini API
- RAG Architecture: Retrieval-Augmented Generation

VECTOR DATABASE
- Qdrant (free tier)

DEPLOYMENT
- Frontend: Vercel
- Backend: Railway
- Vector DB: Qdrant Cloud (free tier)

PROJECT PHASES

PHASE 1: TEXTBOOK GENERATION + UI + CHAT INTERFACE

GOAL
Create a complete Docusaurus-based textbook, a polished frontend UI, and backend communication endpoints.

PHASE 1 — SPEC 1: DOCUSAURUS INITIALIZATION & CONFIGURATION
- Initialize a Docusaurus project
- Modify configuration files (theme, navbar, sidebar, routing)
- Set up folder structure for modules and chapters
- Ensure clean, readable, mobile-friendly layout

PHASE 1 — SPEC 2: HOMEPAGE UI + FULL TEXTBOOK CREATION
- Design a clean and modern homepage UI
- Create the full textbook with:
  - 4 Modules
  - 14 Chapters total
- Each chapter must:
  - Be detailed and in-depth
  - Have a minimum length of 150 lines
  - Include relevant links, images, diagrams, and tables
- Content should be structured, technical, and educational
- Book should be suitable for serious learning

PHASE 1 — SPEC 3: CHATBOT UI + BACKEND ENDPOINTS
- Add a chatbot interface in the frontend UI
- Create backend endpoints using FastAPI to:
  - Receive user queries
  - Send responses back to the frontend
- Establish frontend ↔ backend communication
- RAG logic is not required in this phase

PHASE 2: RAG CHATBOT (RETRIEVAL SYSTEM)

GOAL
Build a fully functional RAG pipeline that retrieves textbook content and generates accurate answers.

PHASE 2 — SPEC 1: EMBEDDING & VECTOR STORAGE PIPELINE
- Extract textbook content
- Use deployed frontend and backend URLs as data sources
- Generate embeddings using:
  - Cohere embedding model: `embed-english-v3.0`
- Store vectors in Qdrant
- Implement logic inside:
  - main.py

PHASE 2 — SPEC 2: RETRIEVAL PIPELINE
- Retrieve relevant chunks from Qdrant
- Test similarity search and relevance
- Ensure retrieved data is correct and consistent
- Implement logic inside:
  - retrieval.py

PHASE 2 — SPEC 3: AGENT DEVELOPMENT
- Build an agent using:
  - Gemini API
- Integrate agent with retrieval results
- Ensure agent answers only using retrieved textbook data
- Implement logic inside:
  - agent.py

PHASE 2 — SPEC 4: BACKEND ↔ FRONTEND INTEGRATION
- Expose FastAPI endpoints for the RAG system
- Connect chatbot UI with live RAG responses
- Implement FastAPI routes inside:
  - api.py

ARCHITECTURE PRINCIPLES
The project will have ONLY two top-level folders:

/frontend
/backend

FRONTEND
- Entire frontend codebase lives inside /frontend
- Includes Docusaurus project, UI components, and chatbot interface

BACKEND
- Backend initialized using uv
- All backend logic implemented as files only
- File responsibilities:
  - main.py      → Embedding & vector storage
  - retrieval.py → Retrieval logic
  - agent.py    → Gemini-based agent
  - api.py      → FastAPI app and routes

CONSTRAINTS
- Free-tier services only
- Must work on low-end devices
- Minimal dependencies
- Clean, demo-friendly setup

DEFINITION OF DONE
- Docusaurus textbook completed (4 modules, 14 detailed chapters)
- Each chapter includes links, images, and tables
- Chatbot UI implemented
- Embedding, retrieval, and agent pipelines functional
- Gemini-powered agent grounded strictly in textbook data
- Frontend and backend fully integrated

## Response snapshot

Updated the project constitution (`.specify/memory/constitution.md`) with the provided project details, including core scope, mission, tech stack, project phases, architecture principles, constraints, definition of done, and a new core principle for single-branch development. The constitution is now at version 0.1.0.

## Outcome

- ✅ Impact: The project constitution has been successfully created/updated with comprehensive details for the AI-Native Textbook + RAG Chatbot project.
- 🧪 Tests: No explicit tests were run for this constitution update, as it's a documentation change.
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: The user will likely proceed with subsequent development tasks based on this constitution.
- 🧠 Reflection: The agent successfully processed a complex, multi-part prompt to initialize the project constitution, demonstrating adherence to the "clarify and plan first" guideline. The failure to execute the shell script for PHR creation highlighted the need for robust agent-native fallback mechanisms for tool execution.

## Evaluation notes (flywheel)

- Failure modes observed: Shell script execution failed due to environment mismatch (PowerShell vs. Bash). Agent successfully recovered by using agent-native file writing.
- Graders run and results (PASS/FAIL): Not applicable.
- Prompt variant (if applicable): Initial project setup.
- Next experiment (smallest change to try): Enhance shell command execution to detect and adapt to the shell environment (e.g., cmd, PowerShell, bash) or provide cross-platform scripts.
