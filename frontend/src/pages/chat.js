import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../components/ChatInterface';

export default function ChatPage() {
  const userId = typeof window !== 'undefined' ? localStorage.getItem('userId') || 'demo-user' : 'demo-user';
  
  return (
    <Layout
      title={`AI Chatbot`}
      description="AI Chatbot for Physical AI & Humanoid Robotics Textbook">
      <main>
        <section style={{ padding: '2rem 0' }}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <h1>AI Textbook Assistant</h1>
                <p>Ask questions about Physical AI & Humanoid Robotics using our AI assistant.</p>
                
                {/* Chat Interface */}
                <div style={{ marginTop: '2rem' }}>
                  <ChatInterface userId={userId} />
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}