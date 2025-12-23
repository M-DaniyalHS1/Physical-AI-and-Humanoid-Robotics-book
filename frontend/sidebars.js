// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2',
      items: ['ros2/intro', 'ros2/installation', 'ros2/basics'],
    },
    {
      type: 'category',
      label: 'Gazebo & Unity',
      items: ['gazebo-unity/intro', 'gazebo-unity/simulation', 'gazebo-unity/integration'],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac',
      items: ['nvidia-isaac/intro', 'nvidia-isaac/setup', 'nvidia-isaac/examples'],
    },
    {
      type: 'category',
      label: 'VLA (Vision-Language-Action)',
      items: ['vla/intro', 'vla/models', 'vla/applications'],
    },
    {
      type: 'category',
      label: 'AI Integration',
      items: ['ai/chatbot', 'ai/personalization', 'ai/translation'],
    },
  ],
};

module.exports = sidebars;