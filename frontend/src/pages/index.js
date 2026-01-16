import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <div className={styles.greetingSection}>
              <h1 className="hero__title">{siteConfig.title}</h1>
              <p className="hero__subtitle">{siteConfig.tagline}</p>
              <div className={styles.introText}>
                <p>Welcome to the future of technical education! This AI-native textbook platform combines traditional learning with cutting-edge artificial intelligence to teach Physical AI & Humanoid Robotics.</p>
              </div>
              <div className={styles.buttons}>
                <Link
                  className="button button--secondary button--lg"
                  to="/docs/intro">
                  Start Reading - 5 min ⏱️
                </Link>
                <Link
                  className="button button--primary button--lg margin-left--md"
                  to="/chat">
                  Meet AI Assistant 🤖
                  </Link>
              </div>
            </div>
          </div>
          <div className="col col--6 text--center">
            <div className={styles.heroImageContainer}>
              <img
                src="/img/robot-book-cover.svg"
                alt="Physical AI and Humanoid Robotics Book Cover"
                className={styles.heroImage}
              />
            </div>
          </div>
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
      description="Bridging the gap between digital AI and physical robots">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container padding-vert--xl">
            <div className="text--center padding-bottom--lg">
              <Heading as="h2">Why This Book Is Different</Heading>
              <p className="subtitle">Experience the next generation of technical learning</p>
            </div>
            <div className="row">
              <div className="col col--4">
                <div className="card-demo">
                  <div className="card">
                    <div className="card__header">
                      <h3>🤖 AI-Powered Learning</h3>
                    </div>
                    <div className="card__body">
                      <p>
                        Interact with our intelligent assistant to get personalized explanations,
                        adaptive learning paths, and instant answers to your questions.
                      </p>
                    </div>
                  </div>
                </div>
              </div>
              <div className="col col--4">
                <div className="card-demo">
                  <div className="card">
                    <div className="card__header">
                      <h3>🌐 Multilingual Support</h3>
                    </div>
                    <div className="card__body">
                      <p>
                        Access content in multiple languages including Urdu translation,
                        making advanced robotics education accessible to a global audience.
                      </p>
                    </div>
                  </div>
                </div>
              </div>
              <div className="col col--4">
                <div className="card-demo">
                  <div className="card">
                    <div className="card__header">
                      <h3>📚 Comprehensive Coverage</h3>
                    </div>
                    <div className="card__body">
                      <p>
                        From fundamentals to advanced topics in Physical AI and Humanoid Robotics,
                        with hands-on examples and real-world applications.
                      </p>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className="padding-vert--lg">
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2 text--center">
                <Heading as="h2">About This Project</Heading>
                <div className={styles.projectInfo}>
                  <p>
                    This AI-native textbook platform represents the future of technical education.
                    By integrating Retrieval-Augmented Generation (RAG) technology, personalization algorithms,
                    and multilingual support, we're creating an immersive learning experience that adapts to each student.
                  </p>
                  <p>
                    Our mission is to democratize access to advanced robotics education by combining traditional
                    textbook pedagogy with AI-driven insights, interactive experiences, and global accessibility.
                  </p>
                  <div className={styles.techHighlights}>
                    <span className="badge badge--primary">RAG Technology</span>
                    <span className="badge badge--secondary margin-left--sm">Personalization</span>
                    <span className="badge badge--success margin-left--sm">Multilingual</span>
                    <span className="badge badge--info margin-left--sm">Interactive</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.testimonials}>
          <div className="container padding-vert--xl">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <Heading as="h3" className="text--center padding-bottom--lg">What Learners Are Saying</Heading>
                <div className="row">
                  <div className="col col--4">
                    <div className="card">
                      <div className="card__body text--center">
                        <p>"An excellent resource for understanding the intersection of AI and robotics. The interactive AI assistant makes learning so much easier!"</p>
                        <small>- Robotics Engineer</small>
                      </div>
                    </div>
                  </div>
                  <div className="col col--4">
                    <div className="card">
                      <div className="card__body text--center">
                        <p>"The multilingual support is a game-changer. Finally, technical content accessible to a wider audience."</p>
                        <small>- AI Researcher</small>
                      </div>
                    </div>
                  </div>
                  <div className="col col--4">
                    <div className="card">
                      <div className="card__body text--center">
                        <p>"The combination of traditional textbook content with AI-powered explanations creates a truly unique learning experience."</p>
                        <small>- Computer Science Student</small>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.ctaSection}>
          <div className="container text--center padding-vert--xl">
            <Heading as="h2">Ready to Explore the Future of Robotics?</Heading>
            <p>Join thousands of learners advancing their knowledge in Physical AI and Humanoid Robotics</p>
            <Link
              className="button button--primary button--lg margin-top--md"
              to="/docs/intro">
              Begin Your Journey
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}