/**
 * Compact Chat Interface Component
 * A streamlined chat interface designed for the AI Copilot widget
 */

import React, { useState, useEffect, useRef } from 'react';
import { useChatService } from '../services/chat';

const CompactChatInterface = ({ sessionId, initialMessages = [], onMessagesChange }) => {
  const [messages, setMessages] = useState(initialMessages);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const {
    sendChatMessage,
    getChatSessionMessages,
    loading,
    error
  } = useChatService();

  // Update parent when messages change
  useEffect(() => {
    if (onMessagesChange) {
      onMessagesChange(messages);
    }
  }, [messages, onMessagesChange]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading || !sessionId) return;

    const userMessage = {
      id: `user-${Date.now()}`,
      sender_type: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };

    // Add user message to UI immediately
    setMessages(prev => [...prev, userMessage]);
    const currentMessage = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await sendChatMessage(sessionId, currentMessage);

      const aiMessage = {
        id: `ai-${Date.now()}`,
        sender_type: 'ai',
        content: response.response,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (err) {
      console.error('Error sending message:', err);

      const errorMessage = {
        id: `error-${Date.now()}`,
        sender_type: 'system',
        content: 'Sorry, there was an error processing your request. Please try again.',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="compact-chat-interface" style={{ display: 'flex', flexDirection: 'column', height: '100%' }}>
      {/* Messages Container */}
      <div
        className="chat-messages"
        style={{
          flex: 1,
          overflowY: 'auto',
          padding: '16px',
          display: 'flex',
          flexDirection: 'column',
          gap: '12px'
        }}
      >
        {messages.length === 0 ? (
          <div
            style={{
              flex: 1,
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              justifyContent: 'center',
              textAlign: 'center',
              color: '#6b7280',
              padding: '20px'
            }}
          >
            <div style={{ fontSize: '40px', marginBottom: '12px' }}>🤖</div>
            <h4 style={{ margin: '0 0 8px 0', color: '#1f2937' }}>How can I help you?</h4>
            <p style={{ margin: 0, fontSize: '14px' }}>
              Ask me anything about Physical AI & Humanoid Robotics
            </p>
          </div>
        ) : (
          messages.map((msg) => (
            <div
              key={msg.id}
              style={{
                alignSelf: msg.sender_type === 'user' ? 'flex-end' : 'flex-start',
                maxWidth: '85%',
                padding: '10px 14px',
                borderRadius: '18px',
                backgroundColor: msg.sender_type === 'user' ? '#2563eb' : '#ffffff',
                color: msg.sender_type === 'user' ? 'white' : '#1f2937',
                boxShadow: '0 1px 2px rgba(0,0,0,0.05)',
                border: msg.sender_type !== 'user' ? '1px solid #e5e7eb' : 'none'
              }}
            >
              <div style={{ whiteSpace: 'pre-wrap', lineHeight: '1.5' }}>
                {msg.content}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div
            style={{
              alignSelf: 'flex-start',
              maxWidth: '85%',
              padding: '10px 14px',
              borderRadius: '18px',
              backgroundColor: '#ffffff',
              color: '#1f2937',
              boxShadow: '0 1px 2px rgba(0,0,0,0.05)',
              border: '1px solid #e5e7eb'
            }}
          >
            <div>Thinking...</div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <form
        onSubmit={handleSendMessage}
        style={{
          padding: '12px',
          backgroundColor: 'white',
          borderTop: '1px solid #e5e7eb',
          display: 'flex',
          gap: '8px'
        }}
      >
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask AI Copilot..."
          disabled={isLoading || !sessionId}
          style={{
            flex: 1,
            padding: '10px 14px',
            borderRadius: '20px',
            border: '1px solid #e5e7eb',
            fontSize: '14px',
            outline: 'none'
          }}
        />
        <button
          type="submit"
          disabled={!inputValue.trim() || isLoading || !sessionId}
          style={{
            padding: '10px 16px',
            borderRadius: '20px',
            backgroundColor: (inputValue.trim() && !isLoading && sessionId) ? '#2563eb' : '#d1d5db',
            color: 'white',
            border: 'none',
            cursor: (inputValue.trim() && !isLoading && sessionId) ? 'pointer' : 'not-allowed',
            fontSize: '14px',
            fontWeight: 500
          }}
        >
          Send
        </button>
      </form>
    </div>
  );
};

export default CompactChatInterface;