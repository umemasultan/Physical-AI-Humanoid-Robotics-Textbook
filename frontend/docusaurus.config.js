// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook for building intelligent embodied systems',
  favicon: 'img/favicon.ico',

  url: 'https://physicalairumanoidroboticstextbook.vercel.app',
  baseUrl: '/',

  organizationName: 'umemasultan',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Custom fields for runtime configuration
  customFields: {
    backendUrl: process.env.BACKEND_URL || undefined,
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.png',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/umemasultan',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Introduction',
                to: '/overview/introduction',
              },
              {
                label: 'Module 1: ROS 2',
                to: '/module-1-ros2',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/module-2-digital-twin',
              },
            ],
          },
          {
            title: 'More Modules',
            items: [
              {
                label: 'Module 3: NVIDIA Isaac',
                to: '/module-3-ai-brain',
              },
              {
                label: 'Module 4: VLA',
                to: '/module-4-vla',
              },
              {
                label: 'Module 5: Sensor Fusion',
                to: '/module-6-sensor-fusion',
              },
              {
                label: 'Module 6: RL Locomotion',
                to: '/module-5-rl-locomotion',
              },
            ],
          },
          {
            title: 'Connect with the Author',
            items: [
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/umema-sultan-385797341/',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/umemasultan',
              },
              {
                label: 'Instagram',
                href: 'https://www.instagram.com/codedreamer123/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Umema Sultan. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'markup', 'csharp', 'json', 'yaml'],
      },
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 3,
      },
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
    }),
};

export default config;
