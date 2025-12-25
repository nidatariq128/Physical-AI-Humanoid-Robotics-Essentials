// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'مODULE 1: بنیادیں',
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
      label: 'مODULE 2: ROS 2 کنٹرول سسٹم',
      items: ['ros2/intro', 'ros2/exercises'],
    },
    {
      type: 'category',
      label: 'مODULE 3: شبیہ سازی',
      items: ['simulation/simulation-comprehensive'],
    },
    {
      type: 'category',
      label: 'مODULE 4: ادراک اور نیویگیشن',
      items: ['perception/perception-navigation-comprehensive'],
    },
    {
      type: 'category',
      label: 'مODULE 5: وژن-زبان-ایکشن سسٹم',
      items: ['vla/vla-comprehensive'],
    },
    {
      type: 'category',
      label: 'مODULE 6: کیپ اسٹون پروجیکٹ',
      items: ['capstone/capstone-comprehensive'],
    },
    {
      type: 'category',
      label: 'مODULE 7: ہارڈ ویئر',
      items: ['hardware/hardware-comprehensive'],
    },
    {
      type: 'category',
      label: 'مODULE 8: پیشرفت',
      items: ['progression/progression-comprehensive'],
    },
    {
      type: 'category',
      label: 'حوالہ جات',
      items: ['bibliography/references'],
    }
  ],
};

module.exports = sidebars;