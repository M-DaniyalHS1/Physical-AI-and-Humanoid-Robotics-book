/**
 * Custom Layout Component
 * Wraps the default layout and adds the AI Copilot widget
 */

import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import AICopilot from '../components/AICopilot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <AICopilot />
    </>
  );
}