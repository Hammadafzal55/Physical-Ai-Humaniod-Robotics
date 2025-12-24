Great news! The server has successfully started on **port 8001**.

The previous error `[Errno 10048]` indicated that port 8000 was already in use by another process on your machine, which is why we switched to 8001.

You can confirm the server is running by seeing the `uvicorn` logs in your terminal, similar to this:

```
INFO:     Started server process [PID]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8001 (Press CTRL+C to quit)
```

**Important:** You must leave the terminal window where the `uvicorn` command is running **open and active**. If you close it or interrupt the process, the server will stop.

**Now, you can interact with your agent:**

*   **To check the API documentation:** Open your web browser and go to `http://localhost:8001/docs`. This page should now load and display the `/chat` endpoint.

*   **To interact with the agent (send a chat message):** Use `curl` from a *different* terminal window. Remember to use port `8001`:

    **1. Send a greeting:**
    ```bash
    curl -X POST "http://localhost:8001/chat" -H "Content-Type: application/json" -d '{"query": "hello"}'
    ```

    **2. Send a textbook-related question:**
    ```bash
    curl -X POST "http://localhost:8001/chat" -H "Content-Type: application/json" -d '{"query": "What is a ROS 2 node?"}'
    ```
    *(Note: For retrieval questions like the one above, ensure your Qdrant instance has data ingested via `python main.py` and that all `COHERE_API_KEY`, `QDRANT_HOST`, `QDRANT_API_KEY`, `TEXTBOOK_URLS` are set in your `.env` file.)*

Please try accessing the `/docs` page and then sending a chat message using the `curl` commands. Let me know the responses you get!