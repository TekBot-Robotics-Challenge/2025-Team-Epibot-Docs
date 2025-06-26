import { defineConfig } from 'vitepress';

// see https://vitepress.dev/reference/site-config for details
export default defineConfig({
  lang: 'en-US',
  title: 'Epibot Docs - TRC 2k25',
  base: process.env.NODE_ENV === 'production' ? '/2025-Team-Epibot-Docs/' : '/',
  description: 'Documentation of our 5-week technical competition with 3 robotics expertise poles.',
  
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
          { text: 'Week 1', link: '/week1/pole-it' },
          { text: 'Week 2', link: '/week2/pole-it' },
          { text: 'Week 3', link: '/week3/pole-it' },
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
                  { text: 'Project Demo', link: '/week1/pole-it/Demo' },
                  { text: 'Testing The Project', link: '/week1/pole-it/GettingStarted' },
                  { text: 'Core Class', link: '/week1/pole-it/Core' },
                  { text: 'CollectorRobot Class', link: '/week1/pole-it/CollectorRobot' },
                  { text: 'SorterRobot Class', link: '/week1/pole-it/SorterRobot' },
                  { text: 'AStarAlgorithm', link: '/week1/pole-it/AStarAlgorithm' },
                  { text: 'Ncurses Class', link: '/week1/pole-it/Ncurses' },
                  { text: 'Error Class', link: '/week1/pole-it/Error' },
                  { text: 'Parser Class', link: '/week1/pole-it/Parser' },
                ]
              },
            ]
          },

          // Pole Electronic
          { 
            text: 'Electronics Pole',
            link: '/week2/pole-electronic/',
            collapsed: true,
            items: [
              { text: 'Test 1', link: '/week1/pole-electronic/' },
            ]
          },

          // Pole Mecanic
          { 
            text: 'Mechanical Pole',
            collapsed: true,
            items: [
              // { text: 'Test 1', link: '/week1/pole-it' },
              { 
                text: 'Test 1',
                link: '/week1/pole-mecanic/Piece1',
                collapsed: true,
                items: [
                  { text: 'Piece 1', link: '/week1/pole-mecanic/Piece1' },
                  { text: 'Piece 2', link: '/week1/pole-mecanic/Piece2' },
                  { text: 'Piece 3', link: '/week1/pole-mecanic/Piece3' },
                  { text: 'Piece 4', link: '/week1/pole-mecanic/Piece4' },
                  { text: 'Gripper Assembly', link: '/week1/pole-mecanic/MechanicalGripper' },
                ]
              },
            ]
          },
        ],
      },
      {
        text: 'Week 2 - Integration',
        collapsed: true,
        items: [
           // Pole IT
          {
            text: 'IT Pole',
            collapsed: true,
            items: [
              // { text: 'Test 1', link: '/week1/pole-it' },
              { 
                text: 'Test 2',
                link: '/week2/pole-it/',
                collapsed: true,
                items: [
                  { text: 'Download ROS2 Humble', link: '/week2/pole-it/Download_ROS2_Humble' },
                  { text: 'Custom Message', link: '/week2/pole-it/custom_message' },
                  { text: 'Publisher Node', link: '/week2/pole-it/publisher' },
                  { text: 'Subscriber Node', link: '/week2/pole-it/Subscriber' },
                  { text: 'Launch File Explaination', link: '/week2/pole-it/launch_file' },
                ]
              },
            ]
          },

          // Electronics Pole
          {
            text: 'Electronics Pole',
            collapsed: true,
            items: [
              // { text: 'Test 1', link: '/week1/pole-it' },
              { 
                text: 'Test 2',
                link: '/week2/pole-electronic/',
                collapsed: true,
                items: [
                  { text: '', link: '/week2/pole-electronic/' },
                ]
              },
            ]
          },

          // Mechanical Pole
          {
            text: 'Mechanical Pole',
            collapsed: true,
            items: [
              // { text: 'Test 1', link: '/week1/pole-it' },
              { 
                text: 'Test 2',
                link: '/week2/pole-mecanic/Part_1',
                collapsed: true,
                items: [
                  {text: 'Part 1', link: '/week2/pole-mecanic/Part_1'},
                  {text: 'Part 2', link: '/week2/pole-mecanic/Part_2'},
                  {text: 'Part 3', link: '/week2/pole-mecanic/Part_3'},
                  {text: 'Assembly', link: '/week2/pole-mecanic/assembly'},
                ]
              },
            ]
          },
        ],
      },

      // WEEK 3

      {
        text: 'Week 3 - Finalization',
        collapsed: true,
        items: [
           // Pole IT
          {
            text: 'IT Pole',
            collapsed: true,
            items: [
              { 
                text: 'Test 3',
                link: '/week3/pole-it/',
                collapsed: true,
                items: [
                  { text: 'Autonomous Mapping (SLAM Toolbox)', link: '/week3/pole-it/slam_toolbox' },
                  { text: 'PathPlanner Node', link: '/week3/pole-it/PathPlannerNode' },
                  { text: 'MotionController Node', link: '/week3/pole-it/MotionControllerNode' },
                  { text: 'Astar Algorithm', link: '/week3/pole-it/AStar' },
                  { text: 'Gazebo', link: '/week3/pole-it/Gazebo' },
                  { text: 'Build', link: '/week3/pole-it/Build' },
                  { text: 'Rviz2', link: '/week3/pole-it/Rviz_documentation' },
                ]
              },
            ]
          },

          // Electronics Pole
          {
            text: 'Electronics Pole',
            collapsed: true,
            items: [
              // { text: 'Test 1', link: '/week1/pole-it' },
              { 
                text: 'Test 3',
                link: '/week3/pole-electronic/',
                collapsed: true,
                items: [
                  { text: '', link: '/week3/pole-electronic/' },
                ]
              },
            ]
          },

          // Mechanical Pole
          {
            text: 'Mechanical Pole',
            collapsed: true,
            items: [
              // { text: 'Test 1', link: '/week1/pole-it' },
              { 
                text: 'Test 3',
                link: '/week3/pole-mecanic/',
                collapsed: true,
                items: [
                  {text: '', link: '/week3/pole-mecanic/'},
                ]
              },
            ]
          },
        ],
      },
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/TekBot-Robotics-Challenge' }
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
