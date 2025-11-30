import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  link?: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Part 1: ROS 2 (Robotic Nervous System)',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Master the middleware connecting AI brains to robot bodies. Build mental models of distributed systems through manual practice and hands-on learning.
      </>
    ),
    link: '/docs/module-1-ros2/module-1-ros2-overview',
  },
  {
    title: 'Part 2: Gazebo & Unity (Digital Twin)',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Create physics simulations and digital twins for safe, scalable robot training. Learn to evaluate simulation realism and apply sim-to-real transfer.
      </>
    ),
    link: '/docs/module-2-gazebo-unity/module-2-gazebo-unity-overview',
  },
  {
    title: 'Part 3: NVIDIA Isaac (AI-Robot Brain)',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Build AI-powered perception and manipulation capabilities. Master photorealistic simulation, VSLAM, navigation, and reinforcement learning.
      </>
    ),
    link: '/docs/module-3-nvidia-isaac/module-3-nvidia-isaac-overview',
  },
  {
    title: 'Part 4: Vision-Language-Action (VLA)',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Design systems that enable LLMs to control physical robots through natural language. Orchestrate accumulated intelligence for autonomous humanoid control.
      </>
    ),
    link: '/docs/module-4-vla/module-4-vla-overview',
  },
];

function Feature({title, Svg, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
        {link && (
          <Link
            className="button button--secondary button--sm margin-top--sm"
            to={link}>
            Learn More →
          </Link>
        )}
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className="text--center padding-horiz--md margin-bottom--lg">
              <Heading as="h2">Course Structure</Heading>
              <p className="text--lg">
                A progressive 4-part journey from foundational robotics to advanced AI-powered humanoid systems
              </p>
            </div>
          </div>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
        <div className="row margin-top--xl">
          <div className="col col--12">
            <div className="text--center padding-horiz--md">
              <Heading as="h2">Why This Course?</Heading>
            </div>
          </div>
        </div>
        <div className="row margin-top--md">
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <Heading as="h3">🎯 Hands-On Learning</Heading>
              <p>
                Build real skills through manual practice and mental model development. No black-box tools—understand how everything works.
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <Heading as="h3">🧠 Decision Frameworks</Heading>
              <p>
                Learn to reason through design choices, not just follow instructions. Develop the thinking skills needed for production robotics.
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <Heading as="h3">🚀 Production Ready</Heading>
              <p>
                Master industry-standard tools: ROS 2, Gazebo, NVIDIA Isaac, and VLA models. Skills that translate directly to real-world projects.
              </p>
            </div>
          </div>
        </div>
        <div className="row margin-top--lg">
          <div className="col col--12">
            <div className="text--center">
              <Link
                className="button button--primary button--lg"
                to="/docs/preface">
                Start Learning - Read the Preface →
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
