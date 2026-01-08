/**
 * Custom Layout Wrapper
 * Adds the floating chat widget to all pages
 */

import React from 'react';
import Layout from '@theme/Layout';
import FloatingChatWidget from '../components/FloatingChatWidget';

export default function CustomLayout(props) {
  return (
    <>
      <Layout {...props} />
      <FloatingChatWidget />
    </>
  );
}