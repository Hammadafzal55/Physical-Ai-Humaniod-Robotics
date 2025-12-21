import React, { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

// Type for feature item
type FeatureItem = {
  title: string;
  imageUrl: string; // image path (PNG, JPG) or SVG
  description: ReactNode;
};

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
    title: ' AI-Powered Intelligence',
    imageUrl: '/img/book.png',
    description: (
      <>
        Leverage NVIDIA Isaac and Vision-Language-Action models for perception, navigation, and manipulation. Deploy AI to Jetson edge devices for autonomous operation.
      </>
    ),
  },
];

// Feature component
function Feature({ title, imageUrl, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img className={styles.featureImage} src={imageUrl} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

// Main homepage features section
export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
