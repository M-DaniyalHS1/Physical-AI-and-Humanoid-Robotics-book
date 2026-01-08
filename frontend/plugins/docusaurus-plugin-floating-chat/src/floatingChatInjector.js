/**
 * Floating Chat Injector
 * This file will be injected into the client to add the floating chat widget
 */

import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import FloatingChatWidget from './FloatingChatWidget';

if (ExecutionEnvironment.canUseDOM) {
  // Dynamically add the floating chat widget to the DOM
  const initFloatingChat = () => {
    // Create a container for the widget
    const widgetContainer = document.createElement('div');
    widgetContainer.setAttribute('id', 'floating-chat-container');
    document.body.appendChild(widgetContainer);

    // Render the widget using ReactDOM
    const renderWidget = () => {
      if (window.Docusaurus) {
        const { render } = window.Docusaurus.require('react-dom');
        const { createElement } = window.Docusaurus.require('react');
        
        render(createElement(FloatingChatWidget), widgetContainer);
      } else {
        // Fallback if Docusaurus object isn't available
        setTimeout(renderWidget, 100);
      }
    };
    
    renderWidget();
  };

  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initFloatingChat);
  } else {
    initFloatingChat();
  }
}