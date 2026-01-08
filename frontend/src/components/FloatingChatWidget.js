/**
 * Floating Chat Widget Component
 * Provides a floating AI assistant that appears on all pages
 */

import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import ChatInterface from './ChatInterface';

const FloatingChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMounted, setIsMounted] = useState(false);
  const location = useLocation();

  // Only mount the widget after component is mounted to prevent SSR issues
  useEffect(() => {
    setIsMounted(true);
  }, []);

  // Close the chat when navigating to the dedicated chat page
  useEffect(() => {
    if (location.pathname === '/chat') {
      setIsOpen(false);
    }
  }, [location.pathname]);

  if (!isMounted) {
    // Render only the toggle button during SSR
    return (
      <div className="floating-chat-toggle">
        <button
          className="floating-chat-button"
          onClick={() => {}}
          aria-label="Open AI Assistant"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#1976d2',
            color: 'white',
            border: 'none',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            cursor: 'pointer',
            zIndex: 1000,
            fontSize: '24px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
        >
          💬
        </button>
      </div>
    );
  }

  return (
    <div className="floating-chat-widget">
      {!isOpen ? (
        <button
          className="floating-chat-button"
          onClick={() => setIsOpen(true)}
          aria-label="Open AI Assistant"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#1976d2',
            color: 'white',
            border: 'none',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            cursor: 'pointer',
            zIndex: 1000,
            fontSize: '24px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
        >
          💬
        </button>
      ) : (
        <div
          className="floating-chat-container"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '400px',
            height: '600px',
            maxHeight: '80vh',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 8px 32px rgba(0,0,0,0.2)',
            zIndex: 1000,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden'
          }}
        >
          <div
            className="floating-chat-header"
            style={{
              backgroundColor: '#1976d2',
              color: 'white',
              padding: '12px 16px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px' }}>AI Textbook Assistant</h3>
            <button
              onClick={() => setIsOpen(false)}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '20px',
                cursor: 'pointer',
                padding: '0',
                width: '24px',
                height: '24px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center'
              }}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
          
          <div style={{ flex: 1, overflow: 'auto' }}>
            <ChatInterface 
              userId={typeof window !== 'undefined' ? localStorage.getItem('userId') || 'demo-user' : 'demo-user'} 
              style={{ height: '100%' }} 
            />
          </div>
        </div>
      )}
    </div>
  );
};

export default FloatingChatWidget;