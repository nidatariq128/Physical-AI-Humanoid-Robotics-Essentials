// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI Robotics Book',
  tagline: 'An academic text on Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics-essentials.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, use '/' for root
  baseUrl: '/',

  // These are still needed for the GitHub links and deployment
  organizationName: 'nidatariq128', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics-Essentials', // Usually your repo name.

  onBrokenLinks: 'warn',
  markdown: {
    mermaid: true,
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
    // @ts-ignore
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/nidatariq128/Physical-AI-Humanoid-Robotics-Essentials/edit/main/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'AI Robotics Book',
        logo: {
          alt: 'AI Robotics Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            href: 'https://github.com/nidatariq128/Physical-AI-Humanoid-Robotics-Essentials',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book Sections',
            items: [
              {
                label: 'Foundations',
                to: '/docs/foundations/intro',
              },
              {
                label: 'ROS 2 Control',
                to: '/docs/ros2/intro',
              },
              {
                label: 'Simulation',
                to: '/docs/simulation/simulation-comprehensive',
              },
              {
                label: 'Perception & Navigation',
                to: '/docs/perception/perception-navigation-comprehensive',
              },
              {
                label: 'VLA Systems',
                to: '/docs/vla/vla-comprehensive',
              },
              {
                label: 'Capstone Project',
                to: '/docs/capstone/capstone-comprehensive',
              },
              {
                label: 'Hardware',
                to: '/docs/hardware/hardware-comprehensive',
              },
              {
                label: 'Progression',
                to: '/docs/progression/progression-comprehensive',
              },
              {
                label: 'References',
                to: '/docs/bibliography/references',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/nidatariq128/Physical-AI-Humanoid-Robotics-Essentials',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;