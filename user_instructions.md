Here are the definitive steps to run the server from your terminal, precisely as you intend:

1.  **Open your terminal and navigate to the `backend` directory** of your project.
    You should run this command in your terminal:
    `cd C:\Users\User\Desktop\hackathon\Physical-Ai-Humaniod-Robotics\backend`

2.  **Ensure your Python virtual environment is activated.**
    If you're using PowerShell, it's typically:
    `.\.venv\Scripts\Activate.ps1`
    Your prompt should change to include `(backend)`.

3.  **Run the `uvicorn` server** using the following command:
    `uvicorn api:app --host 0.0.0.0 --port 8000`

    **Important:** This command will run in the foreground. **Do NOT close this terminal window or interrupt the process (e.g., by pressing Ctrl+C)** while you want the server to be active.

Once the server successfully starts (you'll see `INFO: Uvicorn running on http://0.0.0.0:8000`), you can interact with it:

*   **To check the API documentation:** Open your web browser and go to `http://localhost:8000/docs`. This page should now load and display the `/chat` endpoint.

*   **To interact with the agent (send a chat message):** Use a tool like `curl` from a *different* terminal window. For example:
    ```bash
    curl -X POST "http://localhost:8000/chat" -H "Content-Type: application/json" -d '{"query": "hello"}'
    ```
    You can also try:
    ```bash
    curl -X POST "http://localhost:8000/chat" -H "Content-Type: application/json" -d '{"query": "What is a ROS 2 node?"}'
    ```
    *(Note: For retrieval questions like the one above, ensure your Qdrant instance has data ingested via `python main.py` and that all `COHERE_API_KEY`, `QDRANT_HOST`, `QDRANT_API_KEY`, `TEXTBOOK_URLS` are set in your `.env` file.)*

Please follow these steps carefully and let me know if you can successfully access the `/docs` page and then interact with the agent.