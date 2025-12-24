# Research Findings: Agent Development (Gemini-Powered RAG Agent)

## 1. Gemini API for Chatbot Agent

*   **Decision**: Utilize the Google Gemini API (specifically `gemini-pro` for chat) for implementing the chatbot agent.
*   **Rationale**: The spec explicitly mandates a "Gemini-powered chatbot agent". Gemini Pro offers robust conversational capabilities and supports function calling, which is essential for integrating the retrieval tool.
*   **Alternatives Considered**: (None, as Gemini is mandated)

## 2. Function Calling for Retrieval Tool Integration

*   **Decision**: Integrate the retrieval pipeline (`backend/retrieval.py`'s `retrieve_context` function) as a Gemini API function tool.
*   **Rationale**: The spec requires the agent to access textbook data "only via `@function_tool`". This approach allows the Gemini model to dynamically decide when and how to call the retrieval mechanism based on user queries, grounding its responses in the provided context.
*   **Alternatives Considered**:
    *   **Direct context stuffing**: Manually pre-fetching context and adding it to every Gemini prompt. Rejected because it bypasses Gemini's tool-use capabilities, increases prompt token usage unnecessarily, and does not allow Gemini to decide when retrieval is needed.

## 3. Prompt Engineering for RAG Agent Behavior

*   **Decision**: Employ a fixed, non-overridable system instruction (system prompt) to define the agent's persona, rules (strict RAG, controlled creativity, no external knowledge), and output format. User queries will be injected as conversation history.
*   **Rationale**: This ensures consistent adherence to the AGENT BEHAVIOR RULES (greetings, strict RAG, controlled creativity, no-data handling) and QUALITY CONSTRAINTS (zero hallucinations, no external knowledge). A well-crafted system prompt is crucial for steering LLM behavior.
*   **Alternatives Considered**:
    *   **Dynamic prompt generation**: Generating system instructions based on context or user intent. Rejected for initial implementation to maintain strict control and simplicity, aligning with "fixed, non-overridable system instruction" constraint.

## 4. Controlled Creativity Implementation

*   **Decision**: Implement "controlled creativity" primarily through the system instruction given to the Gemini model, guiding it to rephrase, improve clarity, and add explanatory flow without altering facts or introducing new concepts.
*   **Rationale**: Leveraging the LLM's natural language generation capabilities within a constrained prompt is the most effective way to achieve controlled creativity as defined in the spec.
*   **Alternatives Considered**:
    *   **Post-processing of responses**: Using another LLM or rule-based system to rephrase agent output. Rejected due to added complexity, latency, and potential for introducing errors or losing original meaning.

## 5. FastAPI Endpoint for Chat Interaction

*   **Decision**: Implement a dedicated FastAPI endpoint (`/chat`) in `backend/api.py` to handle user messages and return agent responses.
*   **Rationale**: This provides a standard, scalable interface for the frontend to interact with the RAG agent, decoupling the agent logic from the API layer. It adheres to the ARCHITECTURE & FILE RULES.
*   **Alternatives Considered**:
    *   Integrating chat logic directly into `main.py`: Rejected to maintain separation of concerns and adhere to the constraint of keeping `main.py` minimal and untouched unless absolutely required. `api.py` is designated for API routes for deployment requiring only `api.py` and `agent.py`.
