// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Course',
  tagline: 'Agentic AI-Powered Learning Platform for Robotics Engineering',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Abdul Mateen', // Usually your GitHub org/user name.
  projectName: 'ai-textbook-rag-chatbot', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  
  markdown: {
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  plugins: [],

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/abdulmateen5251/Physical_AI_Humanoid_Robotics_book_chatbot',
          routeBasePath: 'docs',
          showLastUpdateAuthor: false,
          showLastUpdateTime: false,
        },
        blog: false, // Disable blog
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
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
        title: 'Physical AI & Humanoid Robotics Course',
        logo: {
          alt: '',
          src: 'img/ai-native-book.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Course Content',
          },
          {
            href: 'https://github.com/abdulmateen5251/Physical_AI_Humanoid_Robotics_book_chatbot',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'light',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: ROS 2 Fundamentals',
                to: 'docs/module-01-ros2/introduction',
              },
              {
                label: 'Module 2: URDF & Simulation',
                to: '/docs/module-02-gazebo/gazebo-setup',
              },
              {
                label: 'Module 3: Computer Vision',
                to: '/docs/module-03-isaac/isaac-ecosystem',
              },
              {
                label: 'Module 4: LLM Integration',
                to: '/docs/module-04-vla/whisper-integration',
              },
            ],
          },
          
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/abdulmateen5251/Physical_AI_Humanoid_Robotics_book_chatbot',
              },
              {
                label: 'Linkedin',
                href: 'https://www.linkedin.com/in/abdul-mateen-048241275/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()}  Abdul Mateen — Built with ❤️ using Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml'],
      },
    }),
};

module.exports = config;
