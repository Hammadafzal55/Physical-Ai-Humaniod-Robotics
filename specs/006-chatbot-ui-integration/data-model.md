# Data Model: Chatbot

This document defines the data entities used within the chatbot feature. As the feature is primarily UI-focused, the data model is simple and managed within the frontend application's state.

## 1. ChatMessage

-   **Description**: Represents a single message within the chat conversation history. It is a transient entity that exists only in the frontend's state during a user session.
-   **Attributes**:
    -   `id` (`string`): A unique identifier for the message, used for rendering lists in React (e.g., can be generated using `crypto.randomUUID()` or a timestamp).
    -   `content` (`string`): The text content of the message.
    -   `sender` (`'user' | 'assistant'`): A string literal type indicating whether the message originated from the human user or the AI assistant. This is used to apply different styling and alignment in the chat UI.
-   **Relationships**:
    -   A conversation consists of a time-ordered list of `ChatMessage` objects.
-   **Validation Rules**:
    -   `id` must be unique within a single conversation.
    -   `content` must not be empty.
    -   `sender` must be either `'user'` or `'assistant'`.
