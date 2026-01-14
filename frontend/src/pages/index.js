import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning - 5 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="AI-Powered Physical AI & Humanoid Robotics Textbook: Bridging the gap between digital AI and physical robots">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <div className={styles.featureImageWrapper}>
                    <div className={styles.robotIcon}>🤖</div>
                  </div>
                  <h3>AI-Powered Learning</h3>
                  <p>Interact with our AI chatbot that answers questions based on textbook content</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <div className={styles.featureImageWrapper}>
                    <div className={styles.gearIcon}>⚙️</div>
                  </div>
                  <h3>Personalized Content</h3>
                  <p>Content adapts based on your background in software and hardware</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <div className={styles.featureImageWrapper}>
                    <div className={styles.globeIcon}>🌐</div>
                  </div>
                  <h3>Multilingual Support</h3>
                  <p>Available in English and Urdu to expand accessibility</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section style={{ padding: '4rem 0', backgroundColor: '#f6f6f6' }}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 style={{ textAlign: 'center', marginBottom: '2rem' }}>Explore the Four Core Modules</h2>
                <div className="row">
                  <div className="col col--3">
                    <div style={{ textAlign: 'center', padding: '1rem', backgroundColor: 'white', borderRadius: '8px', boxShadow: '0 4px 6px rgba(0,0,0,0.1)' }}>
                      <h3>ROS 2</h3>
                      <p>Robot Operating System version 2</p>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div style={{ textAlign: 'center', padding: '1rem', backgroundColor: 'white', borderRadius: '8px', boxShadow: '0 4px 6px rgba(0,0,0,0.1)' }}>
                      <h3>Gazebo & Unity</h3>
                      <p>Simulation environments for robotics</p>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div style={{ textAlign: 'center', padding: '1rem', backgroundColor: 'white', borderRadius: '8px', boxShadow: '0 4px 6px rgba(0,0,0,0.1)' }}>
                      <h3>NVIDIA Isaac</h3>
                      <p>NVIDIA's robotics platform</p>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div style={{ textAlign: 'center', padding: '1rem', backgroundColor: 'white', borderRadius: '8px', boxShadow: '0 4px 6px rgba(0,0,0,0.1)' }}>
                      <h3>VLA</h3>
                      <p>Vision-Language-Action models</p>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section style={{ padding: '4rem 0' }}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 style={{ textAlign: 'center', marginBottom: '2rem' }}>About This Textbook</h2>
                <p style={{ fontSize: '1.2rem', lineHeight: '1.7' }}>
                  Welcome to the AI-Powered Physical AI & Humanoid Robotics Textbook, an interactive learning platform that bridges the gap between digital AI and physical robots.
                </p>
                <p style={{ fontSize: '1.2rem', lineHeight: '1.7' }}>
                  This textbook is designed to provide an engaging and personalized learning experience through cutting-edge technology and pedagogical approaches.
                </p>
              </div>
            </div>
            <div style={{ textAlign: 'center', marginTop: '3rem' }}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Begin Your Learning Journey
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}