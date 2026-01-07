import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import React, { useState, useEffect } from 'react';

// Reading Progress Component
function ReadingProgress() {
  const [width, setWidth] = useState(0);

  useEffect(() => {
    const handleScroll = () => {
      const scrollTop = window.scrollY;
      const docHeight = document.documentElement.scrollHeight - window.innerHeight;
      const scrollPercent = (scrollTop / docHeight) * 100;
      setWidth(scrollPercent);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <div className="reading-progress-container">
      <div className="reading-progress-bar" style={{ width: `${width}%` }} />
    </div>
  );
}

// SVG Icons for buttons
const BookIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20"/>
    <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z"/>
  </svg>
);

const GithubIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
    <path d="M12 0C5.37 0 0 5.37 0 12c0 5.31 3.435 9.795 8.205 11.385.6.105.825-.255.825-.57 0-.285-.015-1.23-.015-2.235-3.015.555-3.795-.735-4.035-1.41-.135-.345-.72-1.41-1.23-1.695-.42-.225-1.02-.78-.015-.795.945-.015 1.62.87 1.845 1.23 1.08 1.815 2.805 1.305 3.495.99.105-.78.42-1.305.765-1.605-2.67-.3-5.46-1.335-5.46-5.925 0-1.305.465-2.385 1.23-3.225-.12-.3-.54-1.53.12-3.18 0 0 1.005-.315 3.3 1.23.96-.27 1.98-.405 3-.405s2.04.135 3 .405c2.295-1.56 3.3-1.23 3.3-1.23.66 1.65.24 2.88.12 3.18.765.84 1.23 1.905 1.23 3.225 0 4.605-2.805 5.625-5.475 5.925.435.375.81 1.095.81 2.22 0 1.605-.015 2.895-.015 3.3 0 .315.225.69.825.57A12.02 12.02 0 0 0 24 12c0-6.63-5.37-12-12-12z"/>
  </svg>
);

const RobotIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <rect x="4" y="4" width="16" height="16" rx="2"/>
    <rect x="9" y="9" width="6" height="6"/>
    <line x1="9" y1="1" x2="9" y2="4"/>
    <line x1="15" y1="1" x2="15" y2="4"/>
    <line x1="9" y1="20" x2="9" y2="23"/>
    <line x1="15" y1="20" x2="15" y2="23"/>
    <line x1="20" y1="9" x2="23" y2="9"/>
    <line x1="20" y1="14" x2="23" y2="14"/>
    <line x1="1" y1="9" x2="4" y2="9"/>
    <line x1="1" y1="14" x2="4" y2="14"/>
  </svg>
);

const ArrowIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <line x1="5" y1="12" x2="19" y2="12"/>
    <polyline points="12 5 19 12 12 19"/>
  </svg>
);

const ExternalLinkIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M18 13v6a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h6"/>
    <polyline points="15 3 21 3 21 9"/>
    <line x1="10" y1="14" x2="21" y2="3"/>
  </svg>
);

const CheckIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <polyline points="20 6 9 17 4 12"/>
  </svg>
);

const StarIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="currentColor" stroke="currentColor" strokeWidth="2">
    <polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"/>
  </svg>
);

const TwitterIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
    <path d="M18.244 2.25h3.308l-7.227 8.26 8.502 11.24H16.17l-5.214-6.817L4.99 21.75H1.68l7.73-8.835L1.254 2.25H8.08l4.713 6.231zm-1.161 17.52h1.833L7.084 4.126H5.117z"/>
  </svg>
);

const LinkedInIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
    <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z"/>
  </svg>
);

const YouTubeIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
    <path d="M23.498 6.186a3.016 3.016 0 0 0-2.122-2.136C19.505 3.545 12 3.545 12 3.545s-7.505 0-9.377.505A3.017 3.017 0 0 0 .502 6.186C0 8.07 0 12 0 12s0 3.93.502 5.814a3.016 3.016 0 0 0 2.122 2.136c1.871.505 9.376.505 9.376.505s7.505 0 9.377-.505a3.015 3.015 0 0 0 2.122-2.136C24 15.93 24 12 24 12s0-3.93-.502-5.814zM9.545 15.568V8.432L15.818 12l-6.273 3.568z"/>
  </svg>
);

const EmailIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M4 4h16c1.1 0 2 .9 2 2v12c0 1.1-.9 2-2 2H4c-1.1 0-2-.9-2-2V6c0-1.1.9-2 2-2z"/>
    <polyline points="22,6 12,13 2,6"/>
  </svg>
);

const LocationIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M21 10c0 7-9 13-9 13s-9-6-9-13a9 9 0 0 1 18 0z"/>
    <circle cx="12" cy="10" r="3"/>
  </svg>
);

const TimeIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <circle cx="12" cy="12" r="10"/>
    <polyline points="12 6 12 12 16 14"/>
  </svg>
);

// Module data
const modules = [
  {
    id: 'module-1',
    title: 'Module 1: ROS 2 Fundamentals',
    description: 'Master the Robotic Operating System that powers modern robots. Learn nodes, topics, services, actions, and advanced patterns for building intelligent robotic systems.',
    lessons: ['Introduction to Physical AI', 'Physical AI Landscape', 'ROS 2 Fundamentals', 'Python & ROS 2 Integration'],
    icon: '🤖',
    color: '#667eea',
    link: '/docs/Module-1-ROS-2/introduction-to-physical-ai'
  },
  {
    id: 'module-2',
    title: 'Module 2: Robot Simulation',
    description: 'Build and test robots in Gazebo before deploying to real hardware. Create URDF models, simulate physics, and integrate sensors in a safe virtual environment.',
    lessons: ['Understanding URDF for Humanoids', 'Simulating Robots in Gazebo', 'Advanced Unity Simulation', 'Simulating Robot Senses'],
    icon: '🎮',
    color: '#f5576c',
    link: '/docs/Module-2-Gazebo-Unity/understanding-urdf-for-humanoids'
  },
  {
    id: 'module-3',
    title: 'Module 3: NVIDIA Isaac & VLA',
    description: 'Leverage NVIDIA Isaac and Vision-Language-Action models for perception, navigation, and manipulation. Deploy AI to Jetson edge devices for autonomous operation.',
    lessons: ['NVIDIA Isaac Sim Introduction', 'Visual SLAM & Navigation', 'Advanced Nav2 Path Planning', 'Voice-to-Action with Whisper'],
    icon: '🧠',
    color: '#4facfe',
    link: '/docs/Module-3-ISAAC/introduction-to-nvidia-isaac-sim'
  }
];

// Why This Book data
const whyBookItems = [
  {
    title: 'Comprehensive Curriculum',
    description: 'From ROS 2 fundamentals to advanced VLA models, this book covers the entire Physical AI and Humanoid Robotics stack in a structured, progressive manner.',
    icon: '📚'
  },
  {
    title: 'Hands-On Projects',
    description: 'Build real robots from simulation to deployment. Every concept is reinforced with practical exercises and capstone projects that you can showcase.',
    icon: '🔧'
  },
  {
    title: 'Industry-Relevant',
    description: 'Learn the exact tools and frameworks used by leading robotics companies. Stay ahead with skills that are in high demand.',
    icon: '🚀'
  },
  {
    title: 'Expert Guidance',
    description: 'Clear explanations with diagrams, code examples, and best practices gathered from years of industry experience.',
    icon: '💡'
  }
];

// End Goal data
const endGoal = {
  title: 'Your Journey Ends Here: Build Autonomous Humanoid Robots',
  description: 'Upon completing this comprehensive curriculum, you will have the skills to design, simulate, and deploy autonomous humanoid robots capable of performing complex tasks through AI-powered perception and decision-making.',
  image: '🏆'
};

// What you need vs What you'll gain
const comparisonData = {
  need: [
    { icon: '💻', text: 'A computer with Ubuntu 20.04+ or Windows with WSL2' },
    { icon: '🧠', text: 'Basic programming knowledge (Python preferred)' },
    { icon: '📖', text: 'Willingness to learn and experiment' },
    { icon: '⏰', text: 'Dedication: 2-3 hours per week for 16 weeks' }
  ],
  gain: [
    { icon: '🎓', text: 'Full-stack Physical AI & Robotics expertise' },
    { icon: '🛠️', text: 'Portfolio of 4+ production-ready projects' },
    { icon: '💼', text: 'Skills applicable to $100B+ robotics industry' },
    { icon: '🌐', text: 'Network with community of robotics enthusiasts' },
    { icon: '📜', text: 'Certificate of completion' }
  ]
};

// Floating orb component
function FloatingOrb({ className, color }: { className: string; color: string }) {
  return (
    <div className={clsx(styles.orb, className)} style={{ background: color }}>
      <div className={styles.orbInner} />
    </div>
  );
}

// Statistics section
function HeroStats() {
  const stats = [
    { number: '4', label: 'Modules' },
    { number: '20+', label: 'Lessons' },
    { number: '∞', label: 'Possibilities' },
  ];

  return (
    <div className={styles.statsContainer}>
      {stats.map((stat, index) => (
        <div key={index} className={styles.statCard}>
          <span className={styles.statNumber}>{stat.number}</span>
          <span className={styles.statLabel}>{stat.label}</span>
        </div>
      ))}
    </div>
  );
}

// Module Card Component
function ModuleCard({ module }: { module: typeof modules[0] }) {
  return (
    <Link to={module.link} className={styles.moduleCard} style={{ '--card-color': module.color } as React.CSSProperties}>
      <div className={styles.moduleCardHeader}>
        <span className={styles.moduleIcon}>{module.icon}</span>
        <div className={styles.moduleBadge}>Module {module.id.split('-')[1]}</div>
      </div>
      <h3 className={styles.moduleTitle}>{module.title}</h3>
      <p className={styles.moduleDescription}>{module.description}</p>
      <ul className={styles.moduleLessons}>
        {module.lessons.map((lesson, idx) => (
          <li key={idx}>
            <StarIcon />
            {lesson}
          </li>
        ))}
      </ul>
      <div className={styles.moduleFooter}>
        <span className={styles.startLearning}>Start Learning</span>
        <ArrowIcon />
      </div>
    </Link>
  );
}

// Why This Book Card Component
function WhyBookCard({ item, index }: { item: typeof whyBookItems[0]; index: number }) {
  return (
    <div className={styles.whyBookCard} style={{ '--delay': `${index * 0.1}s` } as React.CSSProperties}>
      <div className={styles.whyBookIcon}>{item.icon}</div>
      <h3 className={styles.whyBookTitle}>{item.title}</h3>
      <p className={styles.whyBookDescription}>{item.description}</p>
    </div>
  );
}

// Comparison Card Component
function ComparisonSection() {
  const [activeTab, setActiveTab] = useState<'need' | 'gain'>('need');

  return (
    <div className={styles.comparisonSection}>
      <div className={styles.comparisonTabs}>
        <button
          className={clsx(styles.comparisonTab, activeTab === 'need' && styles.activeTab)}
          onClick={() => setActiveTab('need')}
        >
          What You Need
        </button>
        <button
          className={clsx(styles.comparisonTab, activeTab === 'gain' && styles.activeTab)}
          onClick={() => setActiveTab('gain')}
        >
          What You'll Gain
        </button>
      </div>
      <div className={styles.comparisonContent}>
        {(activeTab === 'need' ? comparisonData.need : comparisonData.gain).map((item, idx) => (
          <div key={idx} className={styles.comparisonItem}>
            <span className={styles.comparisonIcon}>{item.icon}</span>
            <span className={styles.comparisonText}>{item.text}</span>
          </div>
        ))}
      </div>
    </div>
  );
}

// Footer Component
function Footer() {
  const [email, setEmail] = useState('');

  const handleSubscribe = (e: React.FormEvent) => {
    e.preventDefault();
    alert(`Thank you for subscribing with: ${email}`);
    setEmail('');
  };

  return (
    <footer className={styles.footer}>
      <div className={clsx("container", styles.footerContainer)}>
        <div className={styles.footerGrid}>
          {/* Brand Section */}
          <div className={styles.footerBrand}>
            <h3 className={styles.footerLogo}>Physical AI & Humanoid Robotics</h3>
            <p className={styles.footerTagline}>
              Your comprehensive guide to mastering the future of robotics through hands-on learning and practical projects.
            </p>
            <div className={styles.socialLinks}>
              <a href="https://x.com/HammadAfza94484" target="_blank" rel="noopener noreferrer" className={styles.socialLink}>
                <TwitterIcon />
              </a>
              <a href="https://github.com/hammadafzal55/Physical-Ai-Humaniod-Robotics" target="_blank" rel="noopener noreferrer" className={styles.socialLink}>
                <GithubIcon />
              </a>
              <a href="https://www.linkedin.com/in/hammad-afzal-52788b301/" target="_blank" rel="noopener noreferrer" className={styles.socialLink}>
                <LinkedInIcon />
              </a>
              <a href="https://www.youtube.com/@panaversity" target="_blank" rel="noopener noreferrer" className={styles.socialLink}>
                <YouTubeIcon />
              </a>
            </div>
          </div>

          {/* Quick Links */}
          <div className={styles.footerLinks}>
            <h4>Quick Links</h4>
            <ul>
              <li><Link to="/docs/intro">Getting Started</Link></li>
              <li><Link to="/docs/Module-1-ROS-2/introduction-to-physical-ai">ROS 2 Course</Link></li>
              <li><Link to="/docs/Module-2-Gazebo-Unity/understanding-urdf-for-humanoids">Simulation Guide</Link></li>
              <li><Link to="/docs/Module-3-ISAAC/introduction-to-nvidia-isaac-sim">AI Integration</Link></li>
            </ul>
          </div>

          {/* Contact Info */}
          <div className={styles.footerContact}>
            <h4>Contact Us</h4>
            <div className={styles.contactItems}>
              <div className={styles.contactItem}>
                <EmailIcon />
                <span>hammadafzaln@gmail.com</span>
              </div>
              <div className={styles.contactItem}>
                <LocationIcon />
                <span>Karachi Sindh, Pakistan</span>
              </div>
              <div className={styles.contactItem}>
                <TimeIcon />
                <span>Response within 24h</span>
              </div>
            </div>
          </div>

          {/* Newsletter */}
          <div className={styles.footerNewsletter}>
            <h4>Stay Updated</h4>
            <p>Get the latest robotics tutorials and course updates delivered to your inbox.</p>
            <form onSubmit={handleSubscribe} className={styles.subscribeForm}>
              <input
                type="email"
                placeholder="Enter your email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                className={styles.subscribeInput}
                required
              />
              <button type="submit" className={styles.subscribeButton}>
                Subscribe
              </button>
            </form>
          </div>
        </div>

        <div className={styles.footerBottom}>
          <p>&copy; {new Date().getFullYear()} Physical AI & Humanoid Robotics. All rights reserved.</p>
        </div>
      </div>
    </footer>
  );
}

// Hero Section
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <header className={clsx(styles.heroBanner)}>
      <FloatingOrb className={styles.orb1} color="rgba(102, 126, 234, 0.4)" />
      <FloatingOrb className={styles.orb2} color="rgba(240, 147, 251, 0.3)" />
      <FloatingOrb className={styles.orb3} color="rgba(79, 172, 254, 0.35)" />

      <div className={clsx("container", styles.heroContainer)}>
        <div className={styles.heroContent}>
          <div className={styles.badge}>
            <RobotIcon />
            <span>The Future of Robotics</span>
          </div>

          <h1 className={styles.heroTitle}>
            <span className={styles.titlePrefix}>Welcome to</span>
            <br />
            <span className={styles.titleMain}>
              <span className={styles.titleGradient}>{siteConfig.title}</span>
            </span>
          </h1>

          <p className={styles.heroSubtitle}>
            Your comprehensive guide to Physical AI & Humanoid Robotics.
            Master ROS 2, robot simulation, and AI-powered intelligence.
          </p>

          <div className={styles.buttonGroup}>
            <Link
              className={clsx(styles.glassButton, styles.primaryButton)}
              to="/docs/intro">
              <BookIcon />
              <span>Start Learning</span>
              <ArrowIcon />
            </Link>
            <a
              className={clsx(styles.glassButton, styles.secondaryButton)}
              href="https://github.com/hammadafzal55/Physical-Ai-Humaniod-Robotics"
              target="_blank"
              rel="noopener noreferrer">
              <GithubIcon />
              <span>View on GitHub</span>
            </a>
          </div>

          <HeroStats />
        </div>

        <div className={styles.heroImageWrapper}>
          <div className={styles.imageGlass}>
            <img src="/img/book.png" alt="Book Cover" className={styles.bookImage} />
          </div>
          <div className={styles.imageGlow} />
        </div>
      </div>

      <div className={styles.scrollIndicator}>
        <span className={styles.scrollText}>Start Your Journey</span>
        <div className={styles.scrollArrow}>
          <ArrowIcon />
        </div>
      </div>
    </header>
  );
}

// Modules Section
function ModulesSection() {
  return (
    <section className={styles.modulesSection}>
      <div className={styles.modulesBg}>
        <div className={styles.moduleOrb1} />
        <div className={styles.moduleOrb2} />
      </div>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Explore Our Modules</h2>
          <p className={styles.sectionSubtitle}>
            A structured learning path from fundamentals to advanced autonomous systems
          </p>
        </div>
        <div className={styles.modulesGrid}>
          {modules.map((module, idx) => (
            <ModuleCard key={idx} module={module} />
          ))}
        </div>
      </div>
    </section>
  );
}

// Why This Book Section
function WhyBookSection() {
  return (
    <section className={styles.whyBookSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Why This Book?</h2>
          <p className={styles.sectionSubtitle}>
            Everything you need to become a Physical AI & Robotics expert
          </p>
        </div>
        <div className={styles.whyBookGrid}>
          {whyBookItems.map((item, idx) => (
            <WhyBookCard key={idx} item={item} index={idx} />
          ))}
        </div>
      </div>
    </section>
  );
}

// End Goal Section
function EndGoalSection() {
  return (
    <section className={styles.endGoalSection}>
      <div className="container">
        <div className={styles.endGoalContent}>
          <div className={styles.endGoalText}>
            <h2 className={styles.endGoalTitle}>Your Ultimate Goal</h2>
            <p className={styles.endGoalDescription}>{endGoal.description}</p>
            <div className={styles.endGoalFeatures}>
              <div className={styles.endGoalFeature}>
                <CheckIcon />
                <span>Design humanoid robots from scratch</span>
              </div>
              <div className={styles.endGoalFeature}>
                <CheckIcon />
                <span>Simulate and test in Gazebo & Unity</span>
              </div>
              <div className={styles.endGoalFeature}>
                <CheckIcon />
                <span>Deploy AI models to edge devices</span>
              </div>
              <div className={styles.endGoalFeature}>
                <CheckIcon />
                <span>Build autonomous navigation systems</span>
              </div>
            </div>
          </div>
          <div className={styles.endGoalVisual}>
            <div className={styles.endGoalIcon}>{endGoal.image}</div>
          </div>
        </div>
      </div>
    </section>
  );
}

// Main Home Component
export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title} - Physical AI & Humanoid Robotics`}
      description="Your comprehensive guide to Physical AI & Humanoid Robotics. Master ROS 2, robot simulation, and AI-powered intelligence.">
      <ReadingProgress />
      <HomepageHeader />
      <main>
        <ModulesSection />
        <WhyBookSection />
        <EndGoalSection />
        <ComparisonSection />
      </main>
      <Footer />
    </Layout>
  );
}
