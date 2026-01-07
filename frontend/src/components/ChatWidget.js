/**
 * Chat Widget Component
 * Provides an interactive chat interface for users to ask questions about textbook content
 */

import React, { useState, useEffect, useRef } from 'react';

const ChatWidget = ({
  userId,
  selectedText = null,
  contentId = null,
  sessionId,
  messages = [],
  onSendMessage,
  mode = 'general',
  loading = false,
  error = null
}) => {
  const [inputMessage, setInputMessage] = useState('');
  const [chatMode, setChatMode] = useState(mode); // 'general' or 'selected-text-only'
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle sending a message
  const handleSendMessage = async (e) => {
    e.preventDefault();

    if (!inputMessage.trim() || loading || !sessionId) return;

    try {
      await onSendMessage(sessionId, inputMessage);
      setInputMessage('');
    } catch (err) {
      console.error('Error sending message:', err);
    }
  };

  // Handle mode change
  const handleModeChange = (newMode) => {
    if (newMode !== chatMode) {
      setChatMode(newMode);
    }
  };

  // Convert sender_type to sender for UI
  const getSenderClass = (senderType) => {
    if (senderType === 'user') return 'user-message';
    if (senderType === 'ai') return 'ai-message';
    return 'system-message';
  };

  // Format timestamp
  const formatTimestamp = (timestamp) => {
    if (!timestamp) return '';
    const date = new Date(timestamp);
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className="chat-widget">
      <div className="chat-window">
        <div className="chat-header">
          <h3>AI Textbook Assistant</h3>
          <div className="chat-controls">
            <div className="chat-mode-selector">
              <label htmlFor="chat-mode">Mode:</label>
              <select
                id="chat-mode"
                value={chatMode}
                onChange={(e) => handleModeChange(e.target.value)}
                disabled={loading || error?.includes('API is not configured')}
              >
                <option value="general">General</option>
                <option value="selected-text-only">Selected Text Only</option>
              </select>
            </div>
          </div>
        </div>

        {error && error.includes('API is not configured') ? (
          <div className="api-config-error" style={{ padding: '2rem', textAlign: 'center', flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            <div>
              <h3>Chat Service Not Available</h3>
              <p>{error}</p>
              <p>Please contact the administrator to configure the backend API properly.</p>
            </div>
          </div>
        ) : (
          <>
            <div className="chat-messages">
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${getSenderClass(message.sender_type)}`}
                  style={{
                    alignSelf: message.sender_type === 'user' ? 'flex-end' : 'flex-start',
                    backgroundColor: message.sender_type === 'user'
                      ? '#e3f2fd'
                      : message.sender_type === 'ai'
                        ? '#f5f5f5'
                        : '#fff8e1',
                    padding: '8px 12px',
                    borderRadius: '8px',
                    marginBottom: '8px',
                    maxWidth: '80%'
                  }}
                >
                  <div className="message-content">
                    {message.content}
                  </div>
                  <div className="message-timestamp">
                    {formatTimestamp(message.timestamp)}
                  </div>
                </div>
              ))}
              {loading && (
                <div className="loading-indicator" style={{ alignSelf: 'flex-start', padding: '8px' }}>
                  AI is thinking...
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>

            {error && !error.includes('API is not configured') && (
              <div className="chat-error" style={{ color: 'red', padding: '8px', fontSize: '14px' }}>
                {error}
              </div>
            )}

            <form className="chat-input-form" onSubmit={handleSendMessage}>
              <input
                type="text"
                value={inputMessage}
                onChange={(e) => setInputMessage(e.target.value)}
                placeholder={chatMode === 'selected-text-only' && selectedText
                  ? "Ask about the selected text..."
                  : "Ask about the textbook content..."}
                disabled={loading || !sessionId || error?.includes('API is not configured')}
                style={{
                  flex: 1,
                  padding: '10px',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  marginRight: '8px'
                }}
              />
              <button
                type="submit"
                disabled={loading || !inputMessage.trim() || !sessionId || error?.includes('API is not configured')}
                style={{
                  padding: '10px 15px',
                  backgroundColor: '#1976d2',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: 'pointer'
                }}
              >
                Send
              </button>
            </form>
          </>
        )}
      </div>
    </div>
  );
};

export default ChatWidget;