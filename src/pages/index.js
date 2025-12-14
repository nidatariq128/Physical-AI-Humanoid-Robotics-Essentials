import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="AI Robotics Book - An academic text on Physical AI and Humanoid Robotics">
      <main>
        <section className={styles.heroBanner}>
          <div className="container">
            <div className="row">
              <div className="col col--6">
                <h1 className="hero__title">{siteConfig.title}</h1>
                <p className="hero__subtitle">{siteConfig.tagline}</p>
                <p className={styles.heroDescription}>
                  Explore the fascinating world of AI Robotics where artificial intelligence meets physical systems.
                  Learn how robots perceive, think, and act in the real world through this comprehensive guide.
                </p>
                <div className={styles.buttons}>
                  <Link
                    className="button button--primary button--lg margin-right--lg"
                    to="/docs/foundations/introducing-ai-driven-development">
                    Start Learning
                  </Link>
                  <Link
                    className="button button--secondary button--lg"
                    to="/docs/foundations/intro">
                    View Modules
                  </Link>
                </div>
              </div>
              <div className="col col--6">
                <div className={styles.heroImageContainer}>
                  <svg
                    className={styles.heroImage}
                    viewBox="0 0 612 612"
                    xmlns="http://www.w3.org/2000/svg"
                    style={{maxWidth: '100%', height: 'auto', maxHeight: '250px'}}
                  >
                    <rect width="100%" height="100%" fill="#f8f9fa"/>
                    {/* Central robot figure */}
                    <circle cx="306" cy="200" r="40" fill="#2e8555" opacity="0.8"/>
                    {/* Robot head with face */}
                    <circle cx="290" cy="190" r="4" fill="#fff"/>
                    <circle cx="322" cy="190" r="4" fill="#fff"/>
                    <path d="M 295 215 Q 306 225 317 215" stroke="#fff" strokeWidth="2" fill="none"/>
                    {/* Robot body */}
                    <rect x="266" y="240" width="80" height="100" rx="10" fill="#2e8555" opacity="0.8"/>
                    {/* Robot arms */}
                    <rect x="230" y="250" width="36" height="15" rx="7" fill="#2e8555" opacity="0.8"/>
                    <rect x="346" y="250" width="36" height="15" rx="7" fill="#2e8555" opacity="0.8"/>
                    {/* Robot legs */}
                    <rect x="276" y="340" width="20" height="40" rx="5" fill="#2e8555" opacity="0.8"/>
                    <rect x="310" y="340" width="20" height="40" rx="5" fill="#2e8555" opacity="0.8"/>
                    {/* AI/Neural network elements around the robot */}
                    <circle cx="150" cy="150" r="8" fill="#ff6b6b" opacity="0.7"/>
                    <circle cx="460" cy="150" r="8" fill="#ff6b6b" opacity="0.7"/>
                    <circle cx="100" cy="300" r="8" fill="#4ecdc4" opacity="0.7"/>
                    <circle cx="510" cy="300" r="8" fill="#4ecdc4" opacity="0.7"/>
                    <circle cx="150" cy="450" r="8" fill="#45b7d1" opacity="0.7"/>
                    <circle cx="460" cy="450" r="8" fill="#45b7d1" opacity="0.7"/>
                    {/* Connection lines representing neural network */}
                    <line x1="306" y1="200" x2="150" y2="150" stroke="#ff6b6b" strokeWidth="1" opacity="0.5"/>
                    <line x1="306" y1="200" x2="460" y2="150" stroke="#ff6b6b" strokeWidth="1" opacity="0.5"/>
                    <line x1="306" y1="200" x2="100" y2="300" stroke="#4ecdc4" strokeWidth="1" opacity="0.5"/>
                    <line x1="306" y1="200" x2="510" y2="300" stroke="#4ecdc4" strokeWidth="1" opacity="0.5"/>
                    <line x1="306" y1="200" x2="150" y2="450" stroke="#45b7d1" strokeWidth="1" opacity="0.5"/>
                    <line x1="306" y1="200" x2="460" y2="450" stroke="#45b7d1" strokeWidth="1" opacity="0.5"/>
                    {/* Additional tech elements */}
                    <polygon points="306,100 320,130 350,130 325,150 340,180 306,160 272,180 287,150 262,130 292,130" fill="none" stroke="#2e8555" strokeWidth="2" opacity="0.6"/>
                    {/* Background elements */}
                    <circle cx="50" cy="50" r="150" fill="none" stroke="#e0e0e0" strokeWidth="0.5" opacity="0.3"/>
                    <circle cx="562" cy="562" r="150" fill="none" stroke="#e0e0e0" strokeWidth="0.5" opacity="0.3"/>
                  </svg>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.featuresSection}>
          <div className="container padding-vert--lg">
            <div className="row">
              <div className="col col--4">
                <h2>Foundations</h2>
                <p>Learn the core principles of AI Robotics and embodied intelligence, understanding how digital AI systems interact with the physical world.</p>
              </div>
              <div className="col col--4">
                <h2>Implementation</h2>
                <p>Master practical skills with ROS 2, simulation environments, and real-world deployment techniques for building autonomous robotic systems.</p>
              </div>
              <div className="col col--4">
                <h2>Integration</h2>
                <p>Combine vision, language, and action for complete robotic systems with advanced human-robot interaction capabilities.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}