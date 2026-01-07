import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../components/ChatInterface';

function HomepageHeader() {
  return (
    <header className="hero hero--primary">
      <div className="container">
        <h1 className="hero__title">AI-Powered Physical AI & Humanoid Robotics Textbook</h1>
        <p className="hero__subtitle">Bridging the gap between digital AI and physical robots</p>
      </div>
    </header>
  );
}

export default function Home() {
  const userId = typeof window !== 'undefined' ? localStorage.getItem('userId') || 'demo-user' : 'demo-user';
  
  return (
    <Layout
      title={`AI Textbook`}
      description="AI-Powered Physical AI & Humanoid Robotics Textbook">
      <HomepageHeader />
      <main>
        <section style={{ padding: '2rem 0' }}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <h2>AI Textbook Assistant</h2>
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