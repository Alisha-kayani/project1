/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * Create as many sidebars as you want.
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Course sidebar with Parts and Chapters structure
  courseSidebar: [
    'preface',
    {
      type: 'category',
      label: 'Part 1: ROS 2 (Robotic Nervous System)',
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/module-1-ros2-overview',
          label: 'Chapter 1: ROS 2 Overview',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/module-1-ros2-week-1',
          label: 'Chapter 2: ROS 2 Fundamentals',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/module-1-ros2-week-2',
          label: 'Chapter 3: ROS 2 Services and Advanced Patterns',
        },
      ],
    },
    {
      type: 'category',
      label: 'Part 2: Gazebo & Unity (Digital Twin)',
      items: [
        {
          type: 'doc',
          id: 'module-2-gazebo-unity/module-2-gazebo-unity-overview',
          label: 'Chapter 1: Digital Twin Overview',
        },
        {
          type: 'doc',
          id: 'module-2-gazebo-unity/module-2-gazebo-unity-week-1',
          label: 'Chapter 2: Gazebo Simulation Basics',
        },
        {
          type: 'doc',
          id: 'module-2-gazebo-unity/module-2-gazebo-unity-week-2',
          label: 'Chapter 3: Unity Integration and Sim-to-Real',
        },
      ],
    },
    {
      type: 'category',
      label: 'Part 3: NVIDIA Isaac (AI-Robot Brain)',
      items: [
        {
          type: 'doc',
          id: 'module-3-nvidia-isaac/module-3-nvidia-isaac-overview',
          label: 'Chapter 1: AI-Robot Brain Overview',
        },
        {
          type: 'doc',
          id: 'module-3-nvidia-isaac/module-3-nvidia-isaac-week-1',
          label: 'Chapter 2: Isaac Sim and Photorealistic Simulation',
        },
        {
          type: 'doc',
          id: 'module-3-nvidia-isaac/module-3-nvidia-isaac-week-2',
          label: 'Chapter 3: Isaac ROS and AI-Powered Perception',
        },
      ],
    },
    {
      type: 'category',
      label: 'Part 4: Vision-Language-Action (VLA)',
      items: [
        {
          type: 'doc',
          id: 'module-4-vla/module-4-vla-overview',
          label: 'Chapter 1: VLA Overview',
        },
        {
          type: 'doc',
          id: 'module-4-vla/module-4-vla-week-1',
          label: 'Chapter 2: VLA Fundamentals',
        },
        {
          type: 'doc',
          id: 'module-4-vla/module-4-vla-week-2',
          label: 'Chapter 3: LLM-Controlled Robotics',
        },
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid',
      items: [
        'capstone/capstone-overview',
      ],
    },
    'hardware-requirements',
    'lab-architecture',
    'about',
  ],
};

module.exports = sidebars;

