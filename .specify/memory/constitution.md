<!--
Sync Impact Report:
Version change: None → 0.1.0 (Initial versioning)
List of modified principles:
  - Added: SINGLE-BRANCH DEVELOPMENT ONLY
Added sections: PROJECT, CORE SCOPE, MISSION, TECH STACK, PROJECT PHASES, ARCHITECTURE PRINCIPLES, CONSTRAINTS, DEFINITION OF DONE, Governance.
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ⚠ pending
  - .specify/templates/spec-template.md ⚠ pending
  - .specify/templates/tasks-template.md ⚠ pending
  - .specify/templates/commands/sp.adr.toml ⚠ pending
  - .specify/templates/commands/sp.analyze.toml ⚠ pending
  - .specify/templates/commands/sp.checklist.toml ⚠ pending
  - .specify/templates/commands/sp.clarify.toml ⚠ pending
  - .specify/templates/commands/sp.constitution.toml ⚠ pending
  - .specify/templates/commands/sp.git.commit_pr.toml ⚠ pending
  - .specify/templates/commands/sp.implement.toml ⚠ pending
  - .specify/templates/commands/sp.phr.toml ⚠ pending
  - .specify/templates/commands/sp.plan.toml ⚠ pending
  - .specify/templates/commands/sp.reverse-engineer.toml ⚠ pending
  - .specify/templates/commands/sp.specify.toml ⚠ pending
  - .specify/templates/commands/sp.tasks.toml ⚠ pending
  - .specify/templates/commands/sp.taskstoissues.toml ⚠ pending
Follow-up TODOs: TODO(RATIFICATION_DATE): Original adoption date of the constitution.
-->
# Project Constitution: AI-Native Textbook + RAG Chatbot for Physical AI & Humanoid Robotics

This document outlines the core principles, scope, and technical direction for the "AI-Native Textbook + RAG Chatbot for Physical AI & Humanoid Robotics" project.

## 1. Project Overview

### 1.1. Project Name
AI-Native Textbook + RAG Chatbot for Physical AI & Humanoid Robotics

### 1.2. Core Scope
This project is divided into TWO phases for building:
1) Textbook generation with UI and chat interface
2) RAG-based chatbot built on top of the textbook

The project focuses only on:
- Building a complete AI-native textbook
- Building a RAG chatbot grounded strictly in that textbook

### 1.3. Mission Statement
Build a modern, detailed, AI-generated textbook for Physical AI & Humanoid Robotics and then layer a retrieval-augmented chatbot on top of it.
The chatbot must answer questions using only the textbook content.

## 2. Technical Stack

### 2.1. Frontend Technologies
- Docusaurus (documentation-based textbook)
- React (UI components)
- TypeScript
- CSS Modules / Tailwind CSS (lightweight styling)

### 2.2. Backend Technologies
- Python 3.11+
- FastAPI (API layer)
- uv (Python package & project manager)

### 2.3. AI & ML Technologies
- Embedding Model: Cohere `embed-english-v3.0`
- Agent Model: Gemini API
- RAG Architecture: Retrieval-Augmented Generation

### 2.4. Vector Database
- Qdrant (free tier)

### 2.5. Deployment Platforms
- Frontend: Vercel
- Backend: Railway
- Vector DB: Qdrant Cloud (free tier)

## 3. Project Phases & Specifications

### 3.1. Phase 1: Textbook Generation + UI + Chat Interface

**Goal**: Create a complete Docusaurus-based textbook, a polished frontend UI, and backend communication endpoints.

#### 3.1.1. Spec 1: Docusaurus Initialization & Configuration
- Initialize a Docusaurus project
- Modify configuration files (theme, navbar, sidebar, routing)
- Set up folder structure for modules and chapters
- Ensure clean, readable, mobile-friendly layout

#### 3.1.2. Spec 2: Homepage UI + Full Textbook Creation
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

#### 3.1.3. Spec 3: Chatbot UI + Backend Endpoints
- Add a chatbot interface in the frontend UI
- Create backend endpoints using FastAPI to:
  - Receive user queries
  - Send responses back to the frontend
- Establish frontend ↔ backend communication
- RAG logic is not required in this phase

### 3.2. Phase 2: RAG Chatbot (Retrieval System)

**Goal**: Build a fully functional RAG pipeline that retrieves textbook content and generates accurate answers.

#### 3.2.1. Spec 1: Embedding & Vector Storage Pipeline
- Extract textbook content
- Use deployed frontend and backend URLs as data sources
- Generate embeddings using:
  - Cohere embedding model: `embed-english-v3.0`
- Store vectors in Qdrant
- Implement logic inside:
  - `main.py`

#### 3.2.2. Spec 2: Retrieval Pipeline
- Retrieve relevant chunks from Qdrant
- Test similarity search and relevance
- Ensure retrieved data is correct and consistent
- Implement logic inside:
  - `retrieval.py`

#### 3.2.3. Spec 3: Agent Development
- Build an agent using:
  - Gemini API
- Integrate agent with retrieval results
- Ensure agent answers only using retrieved textbook data
- Implement logic inside:
  - `agent.py`

#### 3.2.4. Spec 4: Backend ↔ Frontend Integration
- Expose FastAPI endpoints for the RAG system
- Connect chatbot UI with live RAG responses
- Implement FastAPI routes inside:
  - `api.py`

## 4. Architecture Principles

### 4.1. Top-Level Folder Structure
The project will have ONLY two top-level folders:
- `/frontend`
- `/backend`

**Frontend**:
- Entire frontend codebase lives inside `/frontend`
- Includes Docusaurus project, UI components, and chatbot interface

**Backend**:
- Backend initialized using `uv`
- All backend logic implemented as files only
- File responsibilities:
  - `main.py`      → Embedding & vector storage
  - `retrieval.py` → Retrieval logic
  - `agent.py`    → Gemini-based agent
  - `api.py`      → FastAPI app and routes

## 5. Project Constraints

- Free-tier services only
- Must work on low-end devices
- Minimal dependencies
- Clean, demo-friendly setup

## 6. Definition of Done

- Docusaurus textbook completed (4 modules, 14 detailed chapters)
- Each chapter includes links, images, and tables
- Chatbot UI implemented
- Embedding, retrieval, and agent pipelines functional
- Gemini-powered agent grounded strictly in textbook data
- Frontend and backend fully integrated

## 7. Core Principles

### 7.1. SINGLE-BRANCH DEVELOPMENT ONLY
**Rule**: The entire project must be developed strictly on a single Git branch: `main`.
**Details**:
- ❌ No feature branches
- ❌ No dev, test, experiment, or backup branches
- ❌ No branch switching or parallel branch work
- All commits must be made **directly to the `main` branch**
- This rule applies to:
  - All phases
  - All specs
  - All implementations
**Rationale**: To keep development linear and simple, avoid merge conflicts, and ensure fast iteration and demo stability.

## 8. Governance

### 8.1. Versioning
- **Constitution Version**: 0.1.0
- **Version Increment Rules**:
  - MAJOR: Backward incompatible governance/principle removals or redefinitions.
  - MINOR: New principle/section added or materially expanded guidance.
  - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.

### 8.2. Amendment Procedure
Proposed amendments to this constitution must be submitted as a pull request (if branching were allowed, but for this project, direct updates to `main` are implied) to the project maintainers. Changes require consensus among key stakeholders.

### 8.3. Ratification & Last Amended Dates
- **Ratification Date**: TODO(RATIFICATION_DATE): Original adoption date of the constitution.
- **Last Amended Date**: 2025-12-20