/**
 * Floating Chat Widget Plugin Entry Point
 * This file is loaded globally and renders the floating chat widget
 */

import React from 'react';
import { createRoot } from 'react-dom/client';
import FloatingChatWidget from '../components/FloatingChatWidget';

// Create a container element for the floating chat widget
function initializeFloatingChat() {
  // Check if we're in the browser environment
  if (typeof window !== 'undefined' && typeof document !== 'undefined') {
    // Create a container div for the widget
    let container = document.getElementById('floating-chat-container');
    
    if (!container) {
      container = document.createElement('div');
      container.id = 'floating-chat-container';
      document.body.appendChild(container);
    }

    // Render the floating chat widget
    const root = createRoot(container);
    root.render(<FloatingChatWidget />);
  }
}

// Initialize the floating chat widget when the DOM is ready
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initializeFloatingChat);
} else {
  initializeFloatingChat();
}

// Export nothing since this is just an initializer
export {};