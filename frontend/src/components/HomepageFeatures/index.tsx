import React, { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

// Type for feature item
type FeatureItem = {
  title: string;
  imageUrl: string;
  description: ReactNode;
};

// SVG Icons for features
const RosIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
    <circle cx="12" cy="12" r="3"/>
    <circle cx="19" cy="5" r="2"/>
    <circle cx="19" cy="19" r="2"/>
    <circle cx="5" cy="5" r="2"/>
    <circle cx="5" cy="19" r="2"/>
    <line x1="12" y1="9" x2="12" y2="12"/>
    <line x1="9" y1="12" x2="12" y2="12"/>
    <line x1="15" y1="12" x2="12" y2="12"/>
    <line x1="12" y1="15" x2="12" y2="12"/>
    <line x1="9.5" y1="9.5" x2="11" y2="11"/>
    <line x1="14.5" y1="9.5" x2="13" y2="11"/>
    <line x1="9.5" y1="14.5" x2="11" y2="13"/>
    <line x1="14.5" y1="14.5" x2="13" y2="13"/>
  </svg>
);

const SimIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
    <rect x="2" y="3" width="20" height="14" rx="2"/>
    <line x1="8" y1="21" x2="16" y2="21"/>
    <line x1="12" y1="17" x2="12" y2="21"/>
    <path d="M12 8v4"/>
    <circle cx="12" cy="13" r="2"/>
    <path d="M7 8l2-3"/>
    <path d="M17 8l-2-3"/>
    <path d="M7 16l2 3"/>
    <path d="M17 16l-2 3"/>
  </svg>
);

const AiIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
    <path d="M12 2a4 4 0 0 1 4 4v2a4 4 0 0 1-8 0V6a4 4 0 0 1 4-4z"/>
    <path d="M12 14c-4 0-8 2-8 4v2h16v-2c0-2-4-4-8-4z"/>
    <circle cx="9" cy="10" r="1" fill="currentColor"/>
    <circle cx="15" cy="10" r="1" fill="currentColor"/>
    <path d="M9 13h6"/>
    <path d="M8 6h8"/>
  </svg>
);

// Feature icon wrapper
function FeatureIconWrapper({ icon: Icon, color }: { icon: React.ComponentType; color: string }) {
  return (
    <div className={styles.iconWrapper} style={{ color }}>
      <div className={styles.iconBg}>
        <Icon />
      </div>
    </div>
  );
}

// List of features
const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Fundamentals',
    imageUrl: '/img/ai.png',
    description: (
      <>
        Master the Robotic Operating System that powers modern robots. Learn nodes, topics, services, and advanced patterns for building intelligent robotic systems.
      </>
    ),
  },
  {
    title: 'Robot Simulation',
    imageUrl: '/img/ai2.png',
    description: (
      <>
        Build and test robots in Gazebo before deploying to real hardware. Create URDF models, simulate physics, and integrate sensors in a safe virtual environment.
      </>
    ),
  },
  {
    title: 'AI-Powered Intelligence',
    imageUrl: '/img/book.png',
    description: (
      <>
        Leverage NVIDIA Isaac and Vision-Language-Action models for perception, navigation, and manipulation. Deploy AI to Jetson edge devices for autonomous operation.
      </>
    ),
  },
];

// Feature component
function Feature({ title, description, index }: FeatureItem & { index: number }) {
  const icons = [RosIcon, SimIcon, AiIcon];
  const colors = ['#667eea', '#f5576c', '#4facfe'];
  const Icon = icons[index];

  return (
    <div className={styles.featureCard}>
      <div className={styles.cardGlow} />
      <FeatureIconWrapper icon={Icon} color={colors[index]} />
      <div className={styles.featureContent}>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
      <div className={styles.cardArrow}>
        <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          <line x1="5" y1="12" x2="19" y2="12"/>
          <polyline points="12 5 19 12 12 19"/>
        </svg>
      </div>
    </div>
  );
}

// Main homepage features section
export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className={styles.featuresBg}>
        <div className={styles.featureOrb1} />
        <div className={styles.featureOrb2} />
      </div>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>What You'll Learn</h2>
          <p className={styles.sectionSubtitle}>Comprehensive modules covering all aspects of Physical AI & Humanoid Robotics</p>
        </div>
        <div className="row" style={{ position: 'relative', zIndex: 1 }}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} index={idx} />
          ))}
        </div>
      </div>
    </section>
  );
}
