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
            to="/">
            Physical AI Book by "Ammad Ali"
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
      title={`Hello from ${siteConfig.title}`}
      description="Documentation for Physical AI - Bridging AI and Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h2>ROS 2 Fundamentals</h2>
                <p>Learn how ROS 2 serves as the robotic nervous system for communication, control, and coordination.</p>
              </div>
              <div className="col col--4">
                <h2>Communication Patterns</h2>
                <p>Master topics for continuous data streams and services for request-response interactions.</p>
              </div>
              <div className="col col--4">
                <h2>Python Integration</h2>
                <p>Bridge Python AI agents to ROS 2 using rclpy and understand URDF for robot modeling.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}