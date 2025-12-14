// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: Foundations',
      items: [
        'foundations/intro',
        'foundations/ai-development-revolution',
        'foundations/ai-turning-point',
        'foundations/how-to-make-a-billion-dollars-in-the-ai-era',
        'foundations/nine-pillars-of-ai-driven-development',
        'foundations/introducing-ai-driven-development',
        'foundations/terminology'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 Control System',
      items: ['ros2/intro', 'ros2/exercises'],
    },
    {
      type: 'category',
      label: 'Module 3: Simulation',
      items: ['simulation/simulation-comprehensive'],
    },
    {
      type: 'category',
      label: 'Module 4: Perception and Navigation',
      items: ['perception/perception-navigation-comprehensive'],
    },
    {
      type: 'category',
      label: 'Module 5: Vision-Language-Action Systems',
      items: ['vla/vla-comprehensive'],
    },
    {
      type: 'category',
      label: 'Module 6: Capstone Project',
      items: ['capstone/capstone-comprehensive'],
    },
    {
      type: 'category',
      label: 'Module 7: Hardware',
      items: ['hardware/hardware-comprehensive'],
    },
    {
      type: 'category',
      label: 'Module 8: Progression',
      items: ['progression/progression-comprehensive'],
    },
    {
      type: 'category',
      label: 'References',
      items: ['bibliography/references'],
    }
  ],
};

module.exports = sidebars;