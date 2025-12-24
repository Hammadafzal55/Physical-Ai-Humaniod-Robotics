import os
import asyncio # Added for to_thread
from typing import Optional, Dict, Any, AsyncGenerator # Modified for async generator
from dotenv import load_dotenv
from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI, FunctionTool # Modified imports to include FunctionTool
from openai.types.responses import ResponseTextDeltaEvent # Correct import for streaming event
import json # Added for parsing tool arguments

from retrieval import retrieve_context

# Load environment variables from .env file
load_dotenv()

# Define the core retrieval logic as a separate function
async def _retrieve_context_tool_func(tool_context: Any, params_json_string: str) -> str: # Modified signature to async
    """
    Retrieves relevant text chunks from the textbook based on a query.
    """
    try:
        params = json.loads(params_json_string)
        query = params.get("query")
        top_k = params.get("top_k", 3)
        filters = params.get("filters")

        if not query:
            return "Error: Query parameter is missing for retrieval tool."

        # Run synchronous retrieve_context in a thread pool to make it awaitable
        context = await asyncio.to_thread(
            retrieve_context,
            query=query,
            top_k=top_k,
            filters=filters,
            return_context_string=True
        )
        if context and isinstance(context, str) and context.strip():
            return context
        return "No relevant content was found in the textbook for this query."
    except json.JSONDecodeError:
        return f"Error: Invalid JSON arguments for retrieval tool: {params_json_string}"
    except Exception as e:
        return f"Error executing retrieval tool: {e}"

# Create a FunctionTool instance from the retrieval logic
retrieval_tool_instance = FunctionTool(
    name="retrieval_tool",
    description="Retrieve relevant text chunks from the textbook based on a natural language query.",
    params_json_schema={ # Correct argument name
        "type": "object",
        "properties": {
            "query": {
                "type": "string",
                "description": "The natural language query to retrieve relevant information."
            },
            "top_k": {
                "type": "integer",
                "description": "The number of top relevant text chunks to retrieve.",
                "default": 3
            },
            "filters": {
                "type": "object",
                "description": "Optional filters to apply to the retrieval query (e.g., by chapter, source_url).",
                "additionalProperties": True # Allows for flexible filter keys
            }
        },
        "required": ["query"]
    },
    on_invoke_tool=_retrieve_context_tool_func, # Correct argument name for the callable
    strict_json_schema=False # Disable strict schema validation
)


class GeminiRagAgent:
    """
    A RAG agent powered by Google Gemini and the openai-agents library.
    """
    def __init__(self):
        """
        Initializes the Gemini RAG Agent.
        """
        gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not gemini_api_key:
            raise ValueError("GEMINI_API_KEY environment variable not set.")

        # This client points to the Gemini API using an OpenAI-compatible interface
        client = AsyncOpenAI(
            api_key=gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

        # The agent is configured with a model, instructions, and the retrieval tool.
        self.agent = Agent(
            name="Physical AI and Humanoid Robotics Assistant",
            instructions="You are a helpful assistant for the Physical AI and Humanoid Robotics textbook. Your goal is to provide accurate, helpful, and concise answers. For any question about the textbook's content, you MUST use the provided `retrieval_tool` to find relevant information and base your answer strictly on that information.",
            model=OpenAIChatCompletionsModel(
                model="gemini-2.5-flash", # Set model to gemini-pro
                openai_client=client
            ),
            tools=[retrieval_tool_instance] # Use the FunctionTool instance
        )

    async def chat(self, user_query: str) -> AsyncGenerator[str, None]: # Modified to async generator
        """
        Runs a single turn of the conversation with the user, yielding chunks of the response.
        Handles simple greetings directly.
        """
        lower_query = user_query.lower().strip()

        # Handle simple greetings directly (US1)
        if lower_query in ["hello", "hi"]:
            yield "Hello! How can I help you learn about Physical AI and Humanoid Robotics?"
            return
        if lower_query == "who are you?":
            yield "I am an AI assistant designed to help you understand Physical AI and Humanoid Robotics based on the textbook content."
            return
        
        try:
            # Use Runner.run_streamed for true streaming
            result_stream = Runner.run_streamed(starting_agent=self.agent, input=user_query)
            
            async for event in result_stream.stream_events():
                # Check for the event type that contains text deltas
                if event.type == "raw_response_event" and isinstance(event.data, ResponseTextDeltaEvent):
                    yield event.data.delta
            
        except Exception as e:
            # Yield error message as a chunk
            yield f"An error occurred: {e}"
