# Quickstart: End-to-End Chatbot Testing

This guide provides the steps to run the frontend and backend servers to test the fully integrated chatbot feature.

## Prerequisites

1.  **Backend Configuration**: Ensure you have a valid `.env` file in the `backend/` directory with the following variables correctly set:
    -   `COHERE_API_KEY`
    -   `GEMINI_API_KEY`
    -   `QDRANT_HOST`
    -   `QDRANT_API_KEY`
2.  **Populated Database**: The Qdrant vector database must be populated with the textbook content. If it's empty, run the ingestion script first:
    ```bash
    cd backend
    uv run python main.py
    cd ..
    ```
3.  **Dependencies**: Make sure all dependencies are installed for both the frontend and backend:
    ```bash
    # For backend
    cd backend
    uv sync
    cd ..

    # For frontend
    cd frontend
    npm install
    cd ..
    ```

## Running the Application

### 1. Start the Backend Server

Open a terminal and run the following commands:

```bash
cd backend
uv run uvicorn api:app --reload --port 8000
```

The API server should now be running on `http://localhost:8000`. You can verify it's running by visiting `http://localhost:8000/health` in your browser.

### 2. Start the Frontend Server

Open a **separate** terminal and run the following commands:

```bash
cd frontend
npm start
```

The Docusaurus development server will start, and your browser should open to `http://localhost:3000`.

## Testing the Chatbot

1.  Navigate to the website at `http://localhost:3000`.
2.  In the bottom-right corner of any page, you should see a floating robot icon (🤖).
3.  Click the icon to open the chat interface.
4.  The chat window should appear with a greeting message: "Hello! How can I assist you?".
5.  Type a question related to the textbook content into the input field (e.g., "What is ROS 2?").
6.  Press Enter or click the submit button.
7.  Verify that the agent's response streams into the chat window.
8.  Test the error handling by stopping the backend server and sending another message. The UI should display a user-friendly error.
