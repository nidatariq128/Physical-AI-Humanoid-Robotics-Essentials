// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Módulo 1: Fundamentos',
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
      label: 'Módulo 2: Sistema de Control ROS 2',
      items: ['ros2/intro', 'ros2/exercises'],
    },
    {
      type: 'category',
      label: 'Módulo 3: Simulación',
      items: ['simulation/simulation-comprehensive'],
    },
    {
      type: 'category',
      label: 'Módulo 4: Percepción y Navegación',
      items: ['perception/perception-navigation-comprehensive'],
    },
    {
      type: 'category',
      label: 'Módulo 5: Sistemas Visión-Lenguaje-Acción',
      items: ['vla/vla-comprehensive'],
    },
    {
      type: 'category',
      label: 'Módulo 6: Proyecto Final',
      items: ['capstone/capstone-comprehensive'],
    },
    {
      type: 'category',
      label: 'Módulo 7: Hardware',
      items: ['hardware/hardware-comprehensive'],
    },
    {
      type: 'category',
      label: 'Módulo 8: Progresión',
      items: ['progression/progression-comprehensive'],
    },
    {
      type: 'category',
      label: 'Referencias',
      items: ['bibliography/references'],
    }
  ],
};

module.exports = sidebars;