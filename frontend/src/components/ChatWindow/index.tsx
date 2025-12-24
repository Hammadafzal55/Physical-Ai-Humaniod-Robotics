import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useChat } from '../../context/ChatContext';
import styles from './styles.module.css'; // Assuming CSS Modules for styling

// Define ChatMessage interface based on data-model.md
interface ChatMessage {
  id: string;
  content: string;
  sender: 'user' | 'assistant';
}

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://hammad231-backend.hf.space/' // Replace with your production URL
  : 'http://localhost:8000'; // Default for development

const ChatWindow: React.FC = () => {
  const { isOpen, toggleChat } = useChat();
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!isOpen) {
      setMessages([]); // Clear messages when chat is closed
      setError(null); // Clear any errors
    } else if (messages.length === 0) { // Only add greeting if chat is open and messages are empty
      setMessages([{
        id: 'initial-greeting',
        content: 'Hello! How can I assist you?',
        sender: 'assistant',
      }]);
    }
  }, [isOpen, messages.length]); // messages.length dependency is important here to avoid infinite loop

  // Auto-scroll to the latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = useCallback(async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputMessage.trim() || isLoading) return;

    setError(null);
    const userMessage: ChatMessage = {
      id: Date.now().toString(), // Simple unique ID
      content: inputMessage,
      sender: 'user',
    };
    setMessages((prev) => [...prev, userMessage]);
    setInputMessage('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: userMessage.content }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || `HTTP error! Status: ${response.status}`);
      }

      const reader = response.body?.getReader();
      if (!reader) {
        throw new Error('Failed to get response reader.');
      }

      let assistantResponseContent = '';
      const assistantMessageId = Date.now().toString() + '-agent';
      setMessages((prev) => [
        ...prev,
        { id: assistantMessageId, content: '', sender: 'assistant' },
      ]);

      const decoder = new TextDecoder('utf-8');
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        // Decode the chunk and process potential SSE lines
        const chunk = decoder.decode(value, { stream: true });
        // Handle multiple SSE 'data:' lines in one chunk
        const lines = chunk.split('\n');
        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const jsonStr = line.substring(6); // Remove 'data: ' prefix
              if (jsonStr.trim()) { // Ensure it's not an empty data line
                const data = JSON.parse(jsonStr);
                if (data.error) {
                  throw new Error(data.error);
                }
                if (data.content) {
                  assistantResponseContent += data.content;
                  setMessages((prev) =>
                    prev.map((msg) =>
                      msg.id === assistantMessageId
                        ? { ...msg, content: assistantResponseContent }
                        : msg
                    )
                  );
                }
              }
            } catch (parseError) {
              console.error('Failed to parse SSE data:', parseError, 'Line:', line);
              // Continue to next line or handle parse error
            }
          } else if (line.startsWith('event: end')) {
            break; // End of stream signal
          }
        }
      }
    } catch (err) {
      console.error('Chat API Error:', err);
      setError(err instanceof Error ? err.message : 'An unknown error occurred.');
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id.endsWith('-agent') && msg.content === ''
            ? { ...msg, content: err instanceof Error ? err.message : 'Error generating response.' }
            : msg
        )
      );
    } finally {
      setIsLoading(false);
    }
  }, [inputMessage, isLoading]);


  if (!isOpen) {
    return null; // Don't render if chat is closed
  }

  return (
    <div className={styles.chatWindow}>
      <div className={styles.chatHeader}>
        <span className={styles.chatTitle}>Chatbot Assistant</span>
        <button className={styles.closeButton} onClick={toggleChat} aria-label="Close chatbot">
          ✕
        </button>
      </div>
      <div className={styles.chatMessages}>
        {messages.map((msg) => (
          <div
            key={msg.id}
            className={`${styles.chatMessage} ${
              msg.sender === 'user' ? styles.userMessage : styles.assistantMessage
            }`}
          >
            <p>{msg.content}</p>
          </div>
        ))}
        {isLoading && (
          <div className={`${styles.chatMessage} ${styles.assistantMessage}`}>
            <p>...</p> {/* Simple loading indicator */}
          </div>
        )}
        {error && (
            <div className={`${styles.chatMessage} ${styles.assistantMessage} ${styles.errorMessage}`}>
                <p>Error: {error}</p>
            </div>
        )}
        <div ref={messagesEndRef} />
      </div>
      <form onSubmit={handleSubmit} className={styles.chatInputContainer}>
        <input
          type="text"
          placeholder="Ask a question..."
          className={styles.chatInput}
          value={inputMessage}
          onChange={(e) => setInputMessage(e.target.value)}
          disabled={isLoading}
        />
        <button type="submit" className={styles.sendButton} disabled={isLoading || !inputMessage.trim()}>
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatWindow;

