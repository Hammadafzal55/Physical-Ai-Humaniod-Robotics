# Research: Chatbot UI Integration

This document outlines the research tasks required to successfully implement the end-to-end chatbot feature.

## R-01: Global Docusaurus Component Implementation

-   **Task**: Research the best practice for implementing a persistent, site-wide UI component (the chatbot) in Docusaurus v3.
-   **Hypothesis**: The recommended method is "swizzling" the root `Layout` component. This allows wrapping the entire site's layout with a custom component that can provide a React Context and render the chat UI globally.
-   **Investigation Steps**:
    1.  Consult the official Docusaurus documentation on "Swizzling".
    2.  Review community examples or plugins that implement similar global components (e.g., cookie banners, help widgets).
    3.  Run the `npm run swizzle @docusaurus/theme-classic Layout` command to eject the base `Layout` component and analyze its structure.
-   **Success Criteria**: A confirmed, low-risk technical path for injecting the chatbot UI into every page of the Docusaurus site.

## R-02: End-to-End Streaming Response Implementation

-   **Task**: Investigate and define the technical approach for streaming the agent's response from the backend to the frontend.
-   **Area 1: Backend API (`api.py`)**:
    -   **Question**: How do we implement a `StreamingResponse` in FastAPI?
    -   **Investigation**: Review FastAPI documentation for `StreamingResponse`. The implementation will likely involve creating an `async def` generator function that yields response chunks. This generator will be passed to the `StreamingResponse` object with a `media_type` of `text/event-stream`.
-   **Area 2: Backend Agent (`agent.py`)**:
    -   **Question**: How can the `GeminiRagAgent` be modified to produce a stream of response tokens instead of a single string?
    -   **Investigation**: This requires checking the capabilities of the underlying model and agent library.
        -   Does the Gemini API itself support streaming responses for chat completions? (Yes, the Google AI Python SDK's `GenerativeModel.generate_content` method with `stream=True` does).
        -   Does the `openai-agents` library's `Runner` or `OpenAIChatCompletionsModel` expose this streaming capability? Can `Runner.run_sync` be replaced with an async or streaming equivalent like `Runner.run_async` or `Runner.stream`?
-   **Area 3: Agent Library Blocker**:
    -   **Critical Question**: If the `openai-agents` library does **not** support streaming, what is the fallback plan?
    -   **Investigation**:
        -   **Option A (Workaround)**: Can we bypass the `Runner` for streaming calls and interact directly with the `OpenAIChatCompletionsModel` or the underlying `AsyncOpenAI` client? This might break the structured tool-use paradigm of the library.
        -   **Option B (Degraded Experience)**: If a workaround is not feasible, the implementation must fall back to a non-streaming response. This would fail `FR-011` but still deliver the core feature. This risk must be documented and accepted.
-   **Area 4: Frontend Client**:
    -   **Question**: How does the frontend consume a `text/event-stream`?
    -   **Investigation**: The standard `fetch` API in modern browsers can handle streaming responses. The code will need to get a reader from the response body (`response.body.getReader()`) and decode the chunks of data as they arrive, appending them to the component's state.
-   **Success Criteria**: A documented end-to-end code example or a clear set of implementation steps for streaming. If streaming is blocked by the agent library, a clear explanation of the limitation and a documented fallback plan are required.
