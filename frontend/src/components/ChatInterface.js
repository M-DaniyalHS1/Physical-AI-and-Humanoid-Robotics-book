/**
 * Chat Interface Component
 * Integrates the chat UI with backend API using the chat service
 * Provides a complete chat experience with session management and real-time messaging
 */

import React, { useState, useEffect, useCallback } from 'react';
import { useChatService } from '../services/chat';
import ChatWidget from './ChatWidget';

const ChatInterface = ({ userId, contentId, selectedText = null }) => {
  const {
    createChatSession,
    getChatSessions,
    getChatSessionMessages,
    sendChatMessage,
    loading,
    error
  } = useChatService();

  const [activeSession, setActiveSession] = useState(null);
  const [sessions, setSessions] = useState([]);
  const [messages, setMessages] = useState([]);
  const [chatMode, setChatMode] = useState('general');
  const [initialized, setInitialized] = useState(false);

  // Load existing sessions when component mounts
  useEffect(() => {
    if (userId && !initialized) {
      loadChatSessions();
    }
  }, [userId, initialized]);

  // Load chat sessions for the user
  const loadChatSessions = async () => {
    try {
      const userSessions = await getChatSessions({ limit: 10 });
      setSessions(userSessions);
      
      // If there are existing sessions, load the most recent one
      if (userSessions.length > 0) {
        const mostRecentSession = userSessions[0]; // Assuming sessions are sorted by recency
        setActiveSession(mostRecentSession);
        loadSessionMessages(mostRecentSession.id);
      } else {
        // If no existing sessions, create a new one
        await createNewSession();
      }
      
      setInitialized(true);
    } catch (err) {
      console.error('Error loading chat sessions:', err);
    }
  };

  // Create a new chat session
  const createNewSession = async () => {
    try {
      const sessionData = {
        selectedText: chatMode === 'selected-text-only' ? selectedText : null,
        mode: chatMode
      };
      
      const newSession = await createChatSession(sessionData);
      setActiveSession(newSession);
      setSessions(prev => [newSession, ...prev]); // Add to the beginning of the list
      setMessages([]); // Clear any previous messages
    } catch (err) {
      console.error('Error creating chat session:', err);
    }
  };

  // Load messages for a specific session
  const loadSessionMessages = async (sessionId) => {
    try {
      const sessionMessages = await getChatSessionMessages(sessionId);
      setMessages(sessionMessages);
    } catch (err) {
      console.error(`Error loading messages for session ${sessionId}:`, err);
    }
  };

  // Handle sending a message
  const handleSendMessage = async (sessionId, messageContent) => {
    if (!sessionId || !messageContent.trim()) return;

    try {
      // Add user message to UI immediately
      const userMessage = {
        id: `local-${Date.now()}`,
        sender_type: 'user',
        content: messageContent,
        timestamp: new Date().toISOString()
      };
      
      setMessages(prev => [...prev, userMessage]);

      // Send message to backend
      const response = await sendChatMessage(sessionId, messageContent);
      
      // Add AI response to messages
      const aiMessage = {
        id: `ai-${Date.now()}`,
        sender_type: 'ai',
        content: response.response,
        timestamp: new Date().toISOString()
      };
      
      setMessages(prev => [...prev, aiMessage]);
    } catch (err) {
      console.error('Error sending message:', err);
      
      // Add error message to UI
      const errorMessage = {
        id: `error-${Date.now()}`,
        sender_type: 'system',
        content: 'Sorry, there was an error sending your message. Please try again.',
        timestamp: new Date().toISOString()
      };
      
      setMessages(prev => [...prev, errorMessage]);
    }
  };

  // Handle mode change
  const handleModeChange = (newMode) => {
    if (newMode !== chatMode) {
      setChatMode(newMode);
      
      // If we have an active session, we might want to create a new one with the new mode
      if (activeSession) {
        createNewSession();
      }
    }
  };

  // Handle session selection
  const handleSessionSelect = (session) => {
    setActiveSession(session);
    loadSessionMessages(session.id);
  };

  // Handle creating a new session
  const handleNewSession = () => {
    createNewSession();
  };

  // Render session list
  const renderSessionList = () => {
    if (!sessions || sessions.length === 0) {
      return <div className="no-sessions">No chat sessions yet</div>;
    }

    return (
      <div className="session-list">
        <h4>Previous Sessions</h4>
        <ul>
          {sessions.map(session => (
            <li 
              key={session.id} 
              className={`session-item ${activeSession?.id === session.id ? 'active' : ''}`}
              onClick={() => handleSessionSelect(session)}
            >
              <div className="session-title">
                {session.mode === 'selected-text-only' ? '🔍 ' : '💬 '}
                {new Date(session.session_start).toLocaleDateString()}
              </div>
              <div className="session-meta">
                {session.mode} mode
              </div>
            </li>
          ))}
        </ul>
        <button className="new-session-btn" onClick={handleNewSession}>
          + New Session
        </button>
      </div>
    );
  };

  return (
    <div className="chat-interface">
      <div className="chat-interface-header">
        <h2>AI Textbook Assistant</h2>
        <div className="chat-controls">
          <div className="chat-mode-selector">
            <label htmlFor="chat-mode">Mode:</label>
            <select 
              id="chat-mode"
              value={chatMode} 
              onChange={(e) => handleModeChange(e.target.value)}
              disabled={loading}
            >
              <option value="general">General</option>
              <option value="selected-text-only">Selected Text Only</option>
            </select>
          </div>
        </div>
      </div>

      <div className="chat-interface-body">
        <div className="session-sidebar">
          {renderSessionList()}
        </div>
        
        <div className="chat-main">
          {activeSession ? (
            <ChatWidget 
              userId={userId}
              selectedText={selectedText}
              contentId={contentId}
              sessionId={activeSession.id}
              messages={messages}
              onSendMessage={handleSendMessage}
              mode={activeSession.mode}
              loading={loading}
              error={error}
            />
          ) : (
            <div className="no-session-active">
              <p>Select a session or create a new one to start chatting</p>
              <button onClick={handleNewSession}>Start New Chat</button>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default ChatInterface;