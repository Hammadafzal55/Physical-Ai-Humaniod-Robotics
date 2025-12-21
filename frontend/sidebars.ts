const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System (ROS-2)',
      items: [
        'Module-1-ROS-2/introduction-to-physical-ai',
        'Module-1-ROS-2/physical-ai-landscape',
        'Module-1-ROS-2/ros-2-fundamentals',
        'Module-1-ROS-2/python-and-ros-2',
        
      ],
    },
    {
      type: 'category',
      label: 'Module 2 - Simulation with Gazebo & Unity',
      items: [
        'Module-2-Gazebo-Unity/understanding-urdf-for-humanoids',
        'Module-2-Gazebo-Unity/simulating-robots-in-gazebo',
        'Module-2-Gazebo-Unity/advanced-simulation-with-unity',
        'Module-2-Gazebo-Unity/simulating-a-robot-senses',
      ],
    },
    {
      type: 'category',
      label: 'Module 3 - NVIDIA Isaac',
      items: [
        'Module-3-ISAAC/introduction-to-nvidia-isaac-sim',
        'Module-3-ISAAC/visual-slam-and-navigation-with-isaac-ros',
        'Module-3-ISAAC/advanced-path-planning-with-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action',
      items: [
        'Module-4-VLA/voice-to-action-with-openai-whisper',
        'Module-4-VLA/cognitive-planning-with-llms',
        'Module-4-VLA/capstone-project',
      ],
    },
  ],
};

export default sidebars;
