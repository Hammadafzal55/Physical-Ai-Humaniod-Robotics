import React from 'react';
import { useChat } from '../../context/ChatContext';
import styles from './styles.module.css'; // Assuming CSS Modules for styling

const ChatLauncher: React.FC = () => {
  const { toggleChat } = useChat();

  return (
    <button className={styles.chatLauncher} onClick={toggleChat} aria-label="Open chatbot">
      🤖
    </button>
  );
};

export default ChatLauncher;
