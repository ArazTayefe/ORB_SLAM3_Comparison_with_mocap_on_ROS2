# slam_tools

This ROS2 package provides tools for visualizing and comparing SLAM data with motion capture (mocap) ground truth. It includes launch files for visualizing SLAM paths and mocap data in RViz.

## Features

- Publishes SLAM poses for multiple drones
- Visualizes SLAM and mocap data in RViz
- Launch configuration for easy setup

## Clone and Build Instruction
Change your directory to your ROS2 workspace on the local computer:

```bash
cd ~/'your_ros2_workspace'/src
```
Clone the repository:

```bash
git clone https://github.com/ArazTayefe/ORB_SLAM3_Comparison_with_mocap_on_ROS2
cd ../
colcon build --packages-select slam_tools
source install/setup.bash
```

## Launch Instructions

Then launch the visualization:

```bash
ros2 launch slam_tools slam_with_rviz.launch.py
```
