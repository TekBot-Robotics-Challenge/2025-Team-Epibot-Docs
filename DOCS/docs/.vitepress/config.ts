import { defineConfig } from 'vitepress';

// see https://vitepress.dev/reference/site-config for details
export default defineConfig({
  lang: 'en-US',
  title: 'Epibot Docs - TRC 2k25',
  description: 'Documentation of our 3-week technical competition with 3 robotics expertise poles.',
  
  head: [
    ['link', { rel: 'preconnect', href: 'https://fonts.googleapis.com' }],
    ['link', { rel: 'preconnect', href: 'https://fonts.gstatic.com', crossorigin: '' }],
    ['link', { href: 'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800;900&display=swap', rel: 'stylesheet' }],
    ['script', { src: 'https://code.iconify.design/iconify-icon/1.0.8/iconify-icon.min.js' }],
    ['meta', { name: 'theme-color', content: '#00d4ff' }],
  ],

  themeConfig: {
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Our Team', link: '/equipe' },
      { 
        text: 'Weeks', 
        items: [
          { text: 'Week 1', link: '/week1/pole-dev' },
          { text: 'Week 2', link: '/week2/pole-dev' },
          { text: 'Week 3', link: '/week3/pole-dev' },
        ]
      },
    ],

    sidebar: [
      {
        text: 'Overview',
        items: [
          { text: 'The Challenge', link: '/' },
          { text: 'Our Team', link: '/equipe' },
        ],
      },
      {
        text: 'Week 1 - Prototyping',
        collapsed: true,
        items: [
          // Pole IT
          {
            text: 'IT Pole',
            collapsed: true,
            items: [
              // { text: 'Test 1', link: '/week1/pole-it' },
              { 
                text: 'Test 1',
                link: '/week1/pole-it/',
                collapsed: true,
                items: [
                  { text: 'SorterRobot', link: '/week1/pole-it/SorterRobot' },
                ]
              },
            ]
          },

          // Pole Electronic
          { 
            text: 'Electronics Pole',
            collapsed: true,
            items: [
              { text: 'Overview', link: '/week1/pole-electronic/' },
            ]
          },

          // Pole Mecanic
          { 
            text: 'Mechanical Pole',
            collapsed: true,
            items: [
              { text: 'Overview', link: '/week1/pole-mecanic/' },
              { text: 'Task 1 - CAD', link: '/week1/pole-mecanic/ls' },
              { text: 'Assemblies', link: '/week1/pole-mecanic/assemblages' },
              { text: 'Simulation', link: '/week1/pole-mecanic/simulation' },
            ]
          },
        ],
      },
      {
        text: 'Week 2 - Integration',
        collapsed: true,
        items: [
          { text: 'IT Pole', link: '/week2/pole-dev' },
          { text: 'Electronics Pole', link: '/week2/pole-design' },
          { text: 'Mechanical Pole', link: '/week2/pole-marketing' },
        ],
      },
      {
        text: 'Week 3 - Finalization',
        collapsed: true,
        items: [
          { text: 'IT Pole', link: '/week3/pole-dev' },
          { text: 'Electronics Pole', link: '/week3/pole-design' },
          { text: 'Mechanical Pole', link: '/week3/pole-marketing' },
        ],
      },
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/tekbot-robotics-2k25' }
    ],

    footer: {
      message: '🤖 Tekbot Robotics Challenge 2K25 - Where innovation meets technical excellence',
      copyright: 'Copyright © 2025 @Epibot-Epitech'
    },

    search: {
      provider: 'local',
      options: {
        translations: {
          button: {
            buttonText: 'Search',
            buttonAriaLabel: 'Search the documentation'
          },
          modal: {
            displayDetails: 'Show details',
            resetButtonTitle: 'Reset search',
            backButtonTitle: 'Close search',
            noResultsText: 'No results found',
            footer: {
              selectText: 'to select',
              selectKeyAriaLabel: 'enter',
              navigateText: 'to navigate',
              navigateUpKeyAriaLabel: 'arrow up',
              navigateDownKeyAriaLabel: 'arrow down',
              closeText: 'to close',
              closeKeyAriaLabel: 'escape'
            }
          }
        }
      }
    }
  },

  vite: {
    css: {
      preprocessorOptions: {
        css: {
          additionalData: '@import "./theme/custom.css";'
        }
      }
    }
  },

  markdown: {
    theme: {
      light: 'github-light',
      dark: 'github-dark'
    },
    lineNumbers: true
  }
})
