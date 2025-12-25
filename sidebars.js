// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  physicalAISidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'physical-ai/module-1-ros2/chapter-1-robotic-nervous-system',
        'physical-ai/module-1-ros2/chapter-2-communication-motion',
        'physical-ai/module-1-ros2/chapter-3-code-to-body',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'physical-ai/module-2-digital-twin/chapter-1-digital-twin-reality',
        'physical-ai/module-2-digital-twin/chapter-2-physics-gazebo',
        'physical-ai/module-2-digital-twin/chapter-3-perception-unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'physical-ai/module-3-nvidia-isaac/chapter-1-nvidia-isaac-intelligence',
        'physical-ai/module-3-nvidia-isaac/chapter-2-perception-isaac-sim-ros',
        'physical-ai/module-3-nvidia-isaac/chapter-3-navigation-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision–Language–Action (VLA)',
      items: [
        'physical-ai/module-4-vla/chapter-1-vision-language-action-words-to-motion',
        'physical-ai/module-4-vla/chapter-2-voice-to-action-language-interfaces',
        'physical-ai/module-4-vla/chapter-3-capstone-autonomous-humanoid',
      ],
    },
  ],
};

module.exports = sidebars;