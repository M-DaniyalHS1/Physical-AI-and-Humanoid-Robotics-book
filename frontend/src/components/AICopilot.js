/**
 * AI Copilot Component
 * A floating chat widget similar to the Agent Factory implementation
 * Features corner positioning, keyboard shortcut, and online status indicator
 */

import React, { useState, useEffect, useRef } from 'react';
import CompactChatInterface from './CompactChatInterface';
import { useChatService } from '../services/chat';

const AICopilot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [messages, setMessages] = useState([]);
  const [onlineStatus, setOnlineStatus] = useState('checking...');
  const inputRef = useRef(null);

  const {
    createChatSession,
    getChatSessionMessages,
    sendChatMessage,
    loading,
    error
  } = useChatService();

  // Check online status
  useEffect(() => {
    const checkOnlineStatus = async () => {
      try {
        // Attempt a simple API call to check if backend is accessible
        const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/health`);
        if (response.ok) {
          setOnlineStatus('online');
        } else {
          setOnlineStatus('offline');
        }
      } catch (err) {
        setOnlineStatus('offline');
      }
    };

    checkOnlineStatus();

    // Set up periodic status checks
    const interval = setInterval(checkOnlineStatus, 30000); // Check every 30 seconds
    return () => clearInterval(interval);
  }, []);

  // Handle keyboard shortcut (Cmd+K or Ctrl+K)
  useEffect(() => {
    const handleKeyDown = (event) => {
      // Check for Cmd+K (Mac) or Ctrl+K (Windows/Linux)
      if ((event.metaKey || event.ctrlKey) && event.key === 'k') {
        event.preventDefault();
        setIsOpen(!isOpen);

        // Focus input when opening
        if (!isOpen && inputRef.current) {
          setTimeout(() => {
            inputRef.current.focus();
          }, 100);
        }
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen]);

  // Toggle the copilot
  const toggleCopilot = () => {
    setIsOpen(!isOpen);

    // Focus input when opening
    if (!isOpen && inputRef.current) {
      setTimeout(() => {
        inputRef.current.focus();
      }, 100);
    }
  };

  // Start a new chat session
  const startNewChat = async () => {
    try {
      const sessionData = {
        content: 'Hello, I would like to start a chat session.',
        selected_text: null,
        mode: 'general'
      };

      const response = await createChatSession(sessionData);
      setMessages([]);
      setSessionId(response.session_id);
      return response;
    } catch (err) {
      console.error('Error creating chat session:', err);
      return null;
    }
  };

  // Initialize session when component mounts
  useEffect(() => {
    const initializeSession = async () => {
      const newSession = await startNewChat();
      if (newSession) {
        setSessionId(newSession.session_id);
      }
    };

    initializeSession();
  }, []);

  return (
    <div className="ai-copilot">
      {/* AI Copilot Button */}
      {!isOpen && (
        <button
          className="ai-copilot-button"
          onClick={toggleCopilot}
          aria-label="Open AI Copilot"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#2563eb', // Blue color similar to Agent Factory
            color: 'white',
            border: 'none',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            cursor: 'pointer',
            zIndex: 10000,
            fontSize: '20px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            transition: 'all 0.3s ease'
          }}
        >
          <span style={{ position: 'relative' }}>
            🤖
            {onlineStatus === 'online' && (
              <span
                style={{
                  position: 'absolute',
                  top: '-3px',
                  right: '-3px',
                  width: '12px',
                  height: '12px',
                  backgroundColor: '#10b981',
                  borderRadius: '50%',
                  border: '2px solid white'
                }}
                title="AI Copilot Online"
              />
            )}
          </span>
        </button>
      )}

      {/* AI Copilot Panel */}
      {isOpen && (
        <div
          className="ai-copilot-panel"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '400px',
            height: '500px',
            maxHeight: '80vh',
            backgroundColor: 'white',
            borderRadius: '16px',
            boxShadow: '0 10px 40px rgba(0,0,0,0.25)',
            zIndex: 10000,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            fontFamily: '"Inter", -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Oxygen, Ubuntu, Cantarell, "Open Sans", "Helvetica Neue", sans-serif'
          }}
        >
          {/* Header */}
          <div
            className="ai-copilot-header"
            style={{
              backgroundColor: '#f9fafb',
              borderBottom: '1px solid #e5e7eb',
              padding: '16px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
              <div style={{ fontSize: '20px' }}>🤖</div>
              <h3 style={{ margin: 0, fontSize: '16px', fontWeight: 600, color: '#1f2937' }}>AI Copilot</h3>
              <span
                style={{
                  fontSize: '12px',
                  padding: '2px 8px',
                  borderRadius: '12px',
                  backgroundColor: onlineStatus === 'online' ? '#d1fae5' : '#fee2e2',
                  color: onlineStatus === 'online' ? '#065f46' : '#991b1b',
                  marginLeft: '8px'
                }}
              >
                {onlineStatus}
              </span>
            </div>
            <div style={{ display: 'flex', gap: '12px' }}>
              <button
                onClick={async () => {
                  const newSession = await startNewChat();
                  if (newSession) {
                    setSessionId(newSession.session_id);
                    setMessages([]);
                  }
                }}
                style={{
                  background: 'none',
                  border: 'none',
                  color: '#6b7280',
                  cursor: 'pointer',
                  padding: '4px',
                  borderRadius: '4px'
                }}
                title="New Chat"
              >
                ✕
              </button>
              <button
                onClick={toggleCopilot}
                style={{
                  background: 'none',
                  border: 'none',
                  color: '#6b7280',
                  cursor: 'pointer',
                  padding: '4px',
                  borderRadius: '4px'
                }}
                title="Close"
              >
                ×
              </button>
            </div>
          </div>

          {/* Chat Interface */}
          <div style={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
            <CompactChatInterface
              sessionId={sessionId}
              initialMessages={messages}
              onMessagesChange={setMessages}
            />
          </div>

          {/* Keyboard Shortcut Hint */}
          <div
            style={{
              padding: '0 16px 12px',
              textAlign: 'center',
              fontSize: '12px',
              color: '#9ca3af'
            }}
          >
            Press ⌘K to toggle AI Copilot
          </div>
        </div>
      )}
    </div>
  );
};

export default AICopilot;