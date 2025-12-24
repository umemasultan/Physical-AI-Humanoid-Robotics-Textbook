/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      collapsed: false,
      items: [
        'overview/preface',
        'overview/introduction',
      ],
    },
    {
      type: 'category',
      label: 'Core Modules',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/index',
          label: 'Module 1: ROS 2',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/index',
          label: 'Module 2: Digital Twin',
        },
        {
          type: 'doc',
          id: 'module-3-ai-brain/index',
          label: 'Module 3: NVIDIA Isaac',
        },
        {
          type: 'doc',
          id: 'module-4-vla/index',
          label: 'Module 4: Vision-Language-Action',
        },
      ],
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-6-sensor-fusion/index',
          label: 'Module 5: Sensor Fusion',
        },
        {
          type: 'doc',
          id: 'module-5-rl-locomotion/index',
          label: 'Module 6: RL Locomotion',
        },
      ],
    },
    {
      type: 'category',
      label: 'Conclusion',
      collapsed: false,
      items: [
        'conclusion/future-of-physical-ai',
      ],
    },
  ],
};

export default sidebars;
