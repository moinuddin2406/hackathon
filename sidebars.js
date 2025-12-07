/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System',
      items: [
        'module1/robotic-nervous-system-ros2',
      ],
    },
    // TODO: Uncomment when these chapters are created
    // {
    //   type: 'category',
    //   label: 'Module 2: Digital Twin',
    //   items: [
    //     'module2/digital-twin-gazebo-unity',
    //   ],
    // },
    // {
    //   type: 'category',
    //   label: 'Module 3: AI-Robot Brain',
    //   items: [
    //     'module3/ai-robot-brain-nvidia-isaac',
    //   ],
    // },
    // {
    //   type: 'category',
    //   label: 'Module 4: Vision-Language-Action',
    //   items: [
    //     'module4/vision-language-action-vla',
    //   ],
    // },
  ],
};

module.exports = sidebars;
