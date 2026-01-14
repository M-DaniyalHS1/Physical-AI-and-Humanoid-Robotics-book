import React from 'react';
import Layout from '@theme/Layout';

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
                <h2>Introduction</h2>
                <p>Welcome to the AI-Powered Physical AI & Humanoid Robotics Textbook, an interactive learning platform that bridges the gap between digital AI and physical robots.</p>

                <p>This textbook is designed to provide an engaging and personalized learning experience through:</p>
                <ul>
                  <li>AI-powered chatbot that answers questions based on textbook content</li>
                  <li>Personalized content based on your background in software and hardware</li>
                  <li>Multilingual support (English and Urdu)</li>
                  <li>Direct integration with advanced robotics simulation tools (ROS 2, Gazebo, Unity, NVIDIA Isaac)</li>
                </ul>

                <h3>Getting Started</h3>
                <p>To get started with this textbook, explore the different modules:</p>
                <ul>
                  <li><strong>ROS 2</strong>: Learn about Robot Operating System version 2</li>
                  <li><strong>Gazebo & Unity</strong>: Understand simulation environments for robotics</li>
                  <li><strong>NVIDIA Isaac</strong>: Explore NVIDIA's robotics platform</li>
                  <li><strong>VLA (Vision-Language-Action)</strong>: Discover multimodal AI for robotics</li>
                </ul>

                <p>Each section includes interactive elements, code examples, and AI-powered Q&A functionality to enhance your learning experience.</p>

                <p><a href="/docs/intro" className="button button--primary button--lg">Start Learning</a></p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}