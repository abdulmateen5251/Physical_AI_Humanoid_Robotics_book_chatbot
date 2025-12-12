import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.container}>
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <span className={styles.badge}> AI-NATIVE BOOK Physical AI & Humanoid Robotics</span>
            <h1 className={styles.heroTitle}>
              Physical AI & Humanoid Robotics Course
            </h1>
            <p className={styles.heroSubtitle}>
              AI Systems in the Physical World. Embodied Intelligence. Goal:
              Bridging the gap between the digital brain and the physical body.
              Students apply their AI knowledge to control Humanoid Robots in
              simulated and real-world environments
            </p>
            <div className={styles.badges}>
              <span className={styles.badgeItem}>
                <span className={styles.badgeIcon}>✨</span>
                Open Source
              </span>
              <span className={styles.badgeItem}>
                <span className={styles.badgeIcon}>🧡</span>
                Co-Learning with AI
              </span>
              <span className={styles.badgeItem}>
                <span className={styles.badgeIcon}>🔮</span>
                Spec-Driven Development
              </span>
            </div>
            <div className={styles.heroButtons}>
              <Link
                className={styles.buttonPrimary}
                to="/docs/module-01-ros2/introduction">
                Start Reading →
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <img
              src={require('@site/static/img/ai-native-book.png').default}
              alt="AI Native Software Development"
              className={styles.bookImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureSection() {
  return (
    <section className={styles.features}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <div className={styles.featureGrid}>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🤖</div>
            <h3>Module 1: ROS 2 Fundamentals</h3>
            <p>Introduction to ROS 2, nodes, topics, services, and real-time robotics communication</p>
            <Link to="/docs/module-01-ros2/introduction" className={styles.featureLink}>
              Learn More →
            </Link>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🌐</div>
            <h3>Module 2: Digital Twin (Gazebo)</h3>
            <p>Create physics-based simulations, URDF models, and sensor integration with Gazebo</p>
            <Link to="/docs/module-02-gazebo/gazebo-setup" className={styles.featureLink}>
              Learn More →
            </Link>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🧠</div>
            <h3>Module 3: AI-Robot Brain (Isaac)</h3>
            <p>NVIDIA Isaac Sim, synthetic data generation, Nav2 navigation, and sim-to-real transfer</p>
            <Link to="/docs/module-03-isaac/isaac-ecosystem" className={styles.featureLink}>
              Learn More →
            </Link>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>👁️</div>
            <h3>Module 4: Vision-Language-Action</h3>
            <p>Whisper speech recognition, LLM planning, safety validation, and multi-modal AI</p>
            <Link to="/docs/module-04-vla/whisper-integration" className={styles.featureLink}>
              Learn More →
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

function TechStack() {
  return (
    <section className={styles.techStack}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>What You'll Learn</h2>
        <div className={styles.techGrid}>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>⚡</span>
            <span>ROS 2 Humble/Iron</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>🎮</span>
            <span>Gazebo Simulation</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>🎯</span>
            <span>NVIDIA Isaac Sim</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>🗣️</span>
            <span>Whisper Speech AI</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>🤖</span>
            <span>GPT-4 Planning</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>📊</span>
            <span>Nav2 Navigation</span>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="AI-Native Course for Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <FeatureSection />
        <TechStack />
      </main>
    </Layout>
  );
}
