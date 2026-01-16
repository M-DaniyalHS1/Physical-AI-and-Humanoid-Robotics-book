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
        <div className="text--center">
          <div className={styles.tagline}>DIGITAL BRAIN · PHYSICAL BODY</div>
          <h1 className="hero__title">{siteConfig.title}</h1>
          <p className="hero__subtitle">Learn to build embodied intelligence using ROS 2, simulation with Isaac/Gazebo, and modern Vision‑Language‑Action models.</p>
          <div className={styles.ctaRow}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro">
              START LEARNING
            </Link>
            <Link
              className="button button--primary button--lg margin-left--md"
              to="/docs/intro">
              VIEW CURRICULUM
            </Link>
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
        {/* Curriculum Modules Section */}
        <section id="modules" className={styles.modulesSection}>
          <div className="container padding-vert--lg">
            <div className="text--center padding-bottom--lg">
              <Heading as="h2">Curriculum Modules</Heading>
              <p className={styles.sectionDesc}>A four‑part progression from robotics foundations to full embodied AI systems.</p>
            </div>
            <div className="row">
              <div className="col col--3">
                <div className={clsx("card", styles.card)}>
                  <small>01</small>
                  <h3>The Nervous System</h3>
                  <ul>
                    <li>ROS 2 nodes & topics</li>
                    <li>Middleware & DDS</li>
                    <li>Robot communication</li>
                  </ul>
                  <span className={clsx(styles.status, styles.online)}>ONLINE</span>
                </div>
              </div>
              <div className="col col--3">
                <div className={clsx("card", styles.card)}>
                  <small>02</small>
                  <h3>The Digital Twin</h3>
                  <ul>
                    <li>URDF modeling</li>
                    <li>Gazebo / Isaac Sim</li>
                    <li>Physics & sensors</li>
                  </ul>
                  <span className={clsx(styles.status, styles.loading)}>LOADING</span>
                </div>
              </div>
              <div className="col col--3">
                <div className={clsx("card", styles.card)}>
                  <small>03</small>
                  <h3>The Brain</h3>
                  <ul>
                    <li>LLMs for planning</li>
                    <li>Reasoning loops</li>
                    <li>Tool use</li>
                  </ul>
                  <span className={clsx(styles.status, styles.locked)}>LOCKED</span>
                </div>
              </div>
              <div className="col col--3">
                <div className={clsx("card", styles.card)}>
                  <small>04</small>
                  <h3>The Eyes</h3>
                  <ul>
                    <li>Vision models</li>
                    <li>VLA pipelines</li>
                    <li>Spatial awareness</li>
                  </ul>
                  <span className={clsx(styles.status, styles.locked)}>LOCKED</span>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Hardware Requirements Section */}
        <section id="hardware" className={styles.hardwareSection}>
          <div className="container padding-vert--lg">
            <div className="text--center padding-bottom--lg">
              <Heading as="h2">Hardware Requirements</Heading>
              <p className={styles.sectionDesc}>Choose the setup that fits your budget, goals, and access level.</p>
            </div>
            <div className="row">
              <div className="col col--3">
                <div className={clsx("card", styles.card)}>
                  <h3>☁️ Cloud Simulation</h3>
                  <p>Low cost, browser‑based or rented GPUs.</p>
                  <ul>
                    <li>Good for beginners</li>
                    <li>Limited real‑time performance</li>
                  </ul>
                </div>
              </div>
              <div className="col col--3">
                <div className={clsx("card", styles.card)}>
                  <h3>💻 Mid‑Range Station</h3>
                  <p>Recommended personal setup.</p>
                  <ul>
                    <li>RTX‑class GPU</li>
                    <li>Fast iteration</li>
                  </ul>
                </div>
              </div>
              <div className="col col--3">
                <div className={clsx("card", styles.card)}>
                  <h3>🧪 High‑End Lab</h3>
                  <p>Maximum performance.</p>
                  <ul>
                    <li>Multi‑GPU</li>
                    <li>Large simulations</li>
                  </ul>
                </div>
              </div>
              <div className="col col--3">
                <div className={clsx("card", styles.card)}>
                  <h3>🏫 University Lab</h3>
                  <p>Free access if available.</p>
                  <ul>
                    <li>Shared resources</li>
                    <li>Scheduled usage</li>
                  </ul>
                </div>
              </div>
            </div>

            {/* Comparison Table */}
            <div className="padding-vert--lg">
              <table className={styles.table}>
                <thead>
                  <tr>
                    <th>Option</th>
                    <th>Cost</th>
                    <th>Performance</th>
                    <th>Access</th>
                  </tr>
                </thead>
                <tbody>
                  <tr>
                    <td>Cloud</td>
                    <td>Low</td>
                    <td>Medium</td>
                    <td>Any device</td>
                  </tr>
                  <tr>
                    <td>Mid‑Range</td>
                    <td>Medium</td>
                    <td>High</td>
                    <td>Personal</td>
                  </tr>
                  <tr>
                    <td>High‑End</td>
                    <td>High</td>
                    <td>Very High</td>
                    <td>Lab</td>
                  </tr>
                  <tr>
                    <td>University</td>
                    <td>Free</td>
                    <td>High</td>
                    <td>Shared</td>
                  </tr>
                </tbody>
              </table>
            </div>
          </div>
        </section>

        {/* Decision Helper Section */}
        <section id="decision" className={styles.decisionSection}>
          <div className="container padding-vert--lg">
            <div className="text--center padding-bottom--lg">
              <Heading as="h2">Decision Helper</Heading>
              <p className={styles.sectionDesc}>Find the best path based on your situation.</p>
            </div>
            <div className="row">
              <div className="col col--4">
                <div className={clsx("card", styles.card)}>
                  <h3>Student</h3>
                  <p>Use university lab or cloud simulation.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className={clsx("card", styles.card)}>
                  <h3>Solo Developer</h3>
                  <p>Mid‑range workstation recommended.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className={clsx("card", styles.card)}>
                  <h3>Research Group</h3>
                  <p>High‑end lab for large experiments.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}