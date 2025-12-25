// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'الوحدة 1: الأسس',
      items: [
        'foundations/intro',
        'foundations/ai-development-revolution',
        'foundations/ai-turning-point',
        'foundations/terminology',
        'foundations/how-to-make-a-billion-dollars-in-the-ai-era'
      ],
    },
    {
      type: 'category',
      label: 'الوحدة 2: نظام التحكم ROS 2',
      items: ['ros2/intro', 'ros2/exercises'],
    },
    {
      type: 'category',
      label: 'الوحدة 3: المحاكاة',
      items: ['simulation/simulation-comprehensive'],
    },
    {
      type: 'category',
      label: 'الوحدة 4: الإدراك والتنقل',
      items: ['perception/perception-navigation-comprehensive'],
    },
    {
      type: 'category',
      label: 'الوحدة 5: أنظمة الرؤية-اللغة-الحركة',
      items: ['vla/vla-comprehensive'],
    },
    {
      type: 'category',
      label: 'الوحدة 6: مشروع التخرج',
      items: ['capstone/capstone-comprehensive'],
    },
    {
      type: 'category',
      label: 'الوحدة 7: الهاردوير',
      items: ['hardware/hardware-comprehensive'],
    },
    {
      type: 'category',
      label: 'الوحدة 8: التقدم',
      items: ['progression/progression-comprehensive'],
    },
    {
      type: 'category',
      label: 'المراجع',
      items: ['bibliography/references'],
    }
  ],
};

module.exports = sidebars;