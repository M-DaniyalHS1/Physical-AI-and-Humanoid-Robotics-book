/**
 * Custom Layout Component
 * Wraps the default layout and adds the floating chat widget
 */

import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatWidget from '../components/FloatingChatWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatWidget />
    </>
  );
}