# Quickstart: Agent Development (Gemini-Powered RAG Agent)

This document provides a quick guide to understanding and interacting with the Gemini-powered RAG chatbot agent.

## 1. Overview

The Gemini-powered RAG agent is designed to answer questions about the Physical AI & Humanoid Robotics textbook. It handles simple greetings directly and uses a retrieval tool to fetch relevant information from the Qdrant vector database (populated by Spec 3). It answers strictly from retrieved content, allowing limited creativity in explanation.

## 2. Agent Behavior (Summary)

*   **Greetings**: Responds directly to "hello", "hi", "who are you?".
*   **Textbook Questions**: Uses retrieval tool to fetch context, then answers strictly from that context.
*   **Controlled Creativity**: May rephrase and improve clarity without altering facts.
*   **No Data**: Clearly states when an answer is not available in the textbook.

## 3. Chat Endpoint (Conceptual)

The agent will be accessible via a FastAPI `/chat` endpoint.

**Endpoint**: `/chat` (POST)

**Request Body Example**:
```json
{
  "user_message": "What is a ROS 2 node?"
}
```

**Response Body Example**:
```json
{
  "agent_response": "A ROS 2 node is a fundamental building block in a ROS 2 system, serving as an executable process that performs computations. Nodes communicate with each other using topics, services, and actions to achieve complex robotic tasks. (Source: backend/retrieval.py, Chapter: ROS 2 Fundamentals)"
}
```

## 4. Key Technologies

*   **LLM**: Google Gemini API (`gemini-pro`)
*   **Retrieval**: `backend/retrieval.py` (via function tool)
*   **API Framework**: FastAPI
*   **Agent Logic File**: `backend/agent.py`
*   **API Routes File**: `backend/api.py`

## 5. Development Notes

*   Ensure Gemini API key is configured.
*   The `retrieve_context` function from `backend/retrieval.py` will be registered as a function tool for the Gemini model.
*   The system instruction (system prompt) for the Gemini model is crucial for enforcing agent behavior rules and quality constraints.
