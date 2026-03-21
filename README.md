# Multi-Robot ROS2 Gazebo Platform

A multi-robot simulation and navigation platform built with ROS2 Jazzy and Gazebo. This project enables coordinated control and autonomous navigation of multiple differential-drive robots in a simulated environment.

## Prerequisites

Ensure you have ROS2 Jazzy installed on your system. For installation instructions, visit the [ROS2 Jazzy documentation](https://docs.ros.org/en/jazzy/Installation.html).

## Dependencies

Install the required ROS2 packages:

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-twist-mux
sudo apt install ros-jazzy-controller-manager
```

## Workspace Setup

Create and configure your ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## Clone the Repository

```bash
git clone https://github.com/your-repo/multi_robot_platform.git
cd ..
```

## Build the Project

```bash
cd ~/ros2_ws
colcon build
```

## Source the Environment

```bash
source install/setup.bash
```

## Launching the Simulation

### Launch Multi-Robot Simulation

Start the multi-robot simulation with Gazebo and Rviz:

```bash
ros2 launch multi_robot_platform launch_multi_sim.launch.py
```

This will start Gazebo with two differential-drive robots (robot1 and robot2), camera sensors, LiDAR scanners, and the necessary ROS2 control infrastructure.

## Teleoperation

Control each robot independently using the keyboard teleop node:

### Robot 1

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/diff_cont/cmd_vel -p stamped:=true
```

### Robot 2

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot2/diff_cont/cmd_vel -p stamped:=true
```

## Available Topics

The system publishes the following topics:

### Robot 1 & 2 Topics

- `/robot{N}/diff_cont/cmd_vel` - Command velocity (input)
- `/robot{N}/diff_cont/odom` - Odometry
- `/robot{N}/scan` - LiDAR scan data
- `/robot{N}/camera/image_raw` - Camera image
- `/robot{N}/camera/camera_info` - Camera calibration info
- `/robot{N}/joint_states` - Joint state information
- `/robot{N}/map` - Local map
- `/robot{N}/robot_description` - URDF description

### System Topics

- `/tf` - Transformation frames
- `/clock` - Simulation clock

## Architecture

- **Gazebo Integration**: Physics simulation with Gazebo Ignition
- **ROS2 Control**: Hardware abstraction for differential drive control
- **Multi-Robot Support**: Namespaced topics and transforms for each robot
- **Sensing**: Camera and LiDAR sensors on each robot

## Project Structure

```
multi_robot_platform/
├── launch/
│   ├── launch_multi_sim.launch.py    # Multi-robot simulation launcher
│   ├── launch_sim.launch.py          # Single robot simulation
│   └── ...
├── config/
│   ├── my_controllers.yaml           # Controller configuration
│   ├── nav2_params.yaml              # Navigation parameters
│   └── ...
├── description/
│   ├── robot.urdf.xacro              # Robot URDF definition
│   ├── ros2_control.xacro            # Control interface
│   └── ...
├── worlds/
│   └── *.sdf                         # Gazebo world files
└── scripts/
    └── Various utilities and navigation scripts
```
