import pytest
import os
from backend.agent import GeminiRagAgent, retrieval_tool
from unittest.mock import patch, MagicMock

@pytest.fixture
def agent(monkeypatch):
    """Fixture to initialize a GeminiRagAgent instance for tests."""
    monkeypatch.setenv("GEMINI_API_KEY", "test_api_key")
    return GeminiRagAgent()

def test_gemini_agent_initialization(agent):
    """Verify that the agent initializes correctly."""
    assert agent is not None
    assert agent.agent.name == "Physical AI and Humanoid Robotics Assistant"
    assert "helpful assistant" in agent.agent.instructions

def test_greeting_hello(agent):
    """Test direct response to 'hello'."""
    response = agent.chat("hello")
    assert "Hello! How can I help you learn about Physical AI and Humanoid Robotics?" in response

def test_greeting_hi(agent):
    """Test direct response to 'hi'."""
    response = agent.chat("hi")
    assert "Hello! How can I help you learn about Physical AI and Humanoid Robotics?" in response

def test_greeting_who_are_you(agent):
    """Test direct response to 'who are you?'."""
    response = agent.chat("Who are you?") # Test with varied casing
    assert "I am an AI assistant designed to help you understand Physical AI and Humanoid Robotics based on the textbook content." in response

def test_agent_is_configured_with_retrieval_tool(agent):
    """Verify that the retrieval_tool is correctly added to the agent's tools."""
    assert len(agent.agent.tools) == 1
    assert agent.agent.tools[0] == retrieval_tool

def test_textbook_question_simulates_tool_call(agent, monkeypatch):
    """
    Verify that for a textbook question, the agent's tool is called.
    This test manually replaces the tool with a mock and fakes the runner.
    """
    # Arrange: Create a mock for the tool and replace it on the agent instance
    mock_tool = MagicMock(spec=retrieval_tool)
    mock_tool.return_value = "Context about ROS 2 from the textbook."
    agent.agent.tools = [mock_tool]  # Manually inject the mock

    # Arrange: Create a fake run_sync function to simulate the runner's behavior
    def fake_run_sync(starting_agent, input):
        # 1. Simulate the framework calling the registered tool
        tool_result = starting_agent.tools[0](query=input)
        # 2. Simulate the framework getting a final answer from the model
        final_answer = f"Based on the retrieved context: {tool_result}"
        return MagicMock(final_output=final_answer)

    # Arrange: Replace the real run_sync with our fake version for this test
    monkeypatch.setattr("agents.Runner.run_sync", fake_run_sync)

    # Act: Call the chat method with a textbook question
    response = agent.chat("What is ROS 2?")

    # Assert: Verify that our mock tool was called exactly once with the correct query
    mock_tool.assert_called_once_with(query="What is ROS 2?")
    
    # Assert: Verify that the final response includes the context from the tool
    assert "Based on the retrieved context: Context about ROS 2 from the textbook." in response

def test_no_data_retrieval_scenario(agent, monkeypatch):
    """
    Verify that the agent provides a clear message when the retrieval tool
    finds no relevant content.
    """
    # Arrange: Create a mock for the tool that simulates finding no data.
    mock_tool = MagicMock(spec=retrieval_tool)
    mock_tool.return_value = "No relevant content was found in the textbook for this query."
    agent.agent.tools = [mock_tool]  # Manually inject the mock

    # Arrange: Fake the runner. The agent's final response should be based on
    # the tool's output, as per the system instructions.
    def fake_run_sync(starting_agent, input):
        tool_result = starting_agent.tools[0](query=input)
        # The agent is instructed to base its answer strictly on the tool's
        # output. So, it should essentially repeat the "no content" message.
        final_answer = tool_result 
        return MagicMock(final_output=final_answer)

    monkeypatch.setattr("agents.Runner.run_sync", fake_run_sync)

    # Act
    response = agent.chat("Tell me about alien technology.")

    # Assert
    mock_tool.assert_called_once_with(query="Tell me about alien technology.")
    assert "No relevant content was found" in response
