import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/docs',
    component: ComponentCreator('/docs', '446'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'f35'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'da4'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', '70d'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/docs/physical-ai/module-2-digital-twin/',
                component: ComponentCreator('/docs/docs/physical-ai/module-2-digital-twin/', '49c'),
                exact: true
              },
              {
                path: '/docs/physical-ai/module-1-ros2/chapter-1-robotic-nervous-system',
                component: ComponentCreator('/docs/physical-ai/module-1-ros2/chapter-1-robotic-nervous-system', 'd2d'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-1-ros2/chapter-2-communication-motion',
                component: ComponentCreator('/docs/physical-ai/module-1-ros2/chapter-2-communication-motion', 'ef1'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-1-ros2/chapter-3-code-to-body',
                component: ComponentCreator('/docs/physical-ai/module-1-ros2/chapter-3-code-to-body', '8e0'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-2-digital-twin/chapter-1-digital-twin-reality',
                component: ComponentCreator('/docs/physical-ai/module-2-digital-twin/chapter-1-digital-twin-reality', '79b'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-2-digital-twin/chapter-2-physics-gazebo',
                component: ComponentCreator('/docs/physical-ai/module-2-digital-twin/chapter-2-physics-gazebo', '8dd'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-2-digital-twin/chapter-3-perception-unity',
                component: ComponentCreator('/docs/physical-ai/module-2-digital-twin/chapter-3-perception-unity', 'a55'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-2-digital-twin/glossary',
                component: ComponentCreator('/docs/physical-ai/module-2-digital-twin/glossary', '785'),
                exact: true
              },
              {
                path: '/docs/physical-ai/module-3-nvidia-isaac/chapter-1-nvidia-isaac-intelligence',
                component: ComponentCreator('/docs/physical-ai/module-3-nvidia-isaac/chapter-1-nvidia-isaac-intelligence', '252'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-3-nvidia-isaac/chapter-2-perception-isaac-sim-ros',
                component: ComponentCreator('/docs/physical-ai/module-3-nvidia-isaac/chapter-2-perception-isaac-sim-ros', 'a13'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-3-nvidia-isaac/chapter-3-navigation-nav2',
                component: ComponentCreator('/docs/physical-ai/module-3-nvidia-isaac/chapter-3-navigation-nav2', '9e1'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-4-vla/chapter-1-vision-language-action-words-to-motion',
                component: ComponentCreator('/docs/physical-ai/module-4-vla/chapter-1-vision-language-action-words-to-motion', '783'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-4-vla/chapter-2-voice-to-action-language-interfaces',
                component: ComponentCreator('/docs/physical-ai/module-4-vla/chapter-2-voice-to-action-language-interfaces', '15e'),
                exact: true,
                sidebar: "physicalAISidebar"
              },
              {
                path: '/docs/physical-ai/module-4-vla/chapter-3-capstone-autonomous-humanoid',
                component: ComponentCreator('/docs/physical-ai/module-4-vla/chapter-3-capstone-autonomous-humanoid', '787'),
                exact: true,
                sidebar: "physicalAISidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
