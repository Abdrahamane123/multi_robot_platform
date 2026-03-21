# Multi-Robot ROS 2 Platform (Jazzy + Gazebo)

This package provides a multi-robot simulation stack based on ROS 2 Jazzy and Gazebo, with:

- Dynamic spawning of `robot1..robotN`
- Namespaced controllers and sensors
- Optional per-robot SLAM (`slam_toolbox`)
- RViz auto-generated configuration for all robots

## Requirements

- Ubuntu + ROS 2 Jazzy installed
- Gazebo Harmonic / `ros_gz` integration available

Install commonly used dependencies:

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-xacro \
    ros-jazzy-twist-mux \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard
```

## Build

From your workspace root:

```bash
colcon build --packages-select multi_robot_platform
source install/setup.bash
```

## Main Multi-Robot Launch

Start Gazebo + `robot_state_publisher` + controllers + bridges + RViz:

```bash
ros2 launch multi_robot_platform launch_multi_sim.launch.py
```

Supported launch arguments:

- `world` (default: `worlds/empty.world`)
- `robot_count` (default: `2`)
- `robot_spacing` (default: `18.0`)
- `use_slam` (default: `false`)

Example:

```bash
ros2 launch multi_robot_platform launch_multi_sim.launch.py \
    world:=/home/user/ros2_ws/src/multi_robot_platform/worlds/test.sdf \
    robot_count:=2 \
    robot_spacing:=6.0 \
    use_slam:=true
```

## Multi-Robot SLAM

To run `slam_toolbox` for each robot namespace (`robot1..robotN`):

```bash
ros2 launch multi_robot_platform online_async_multi_launch.py robot_count:=2
```

Notes:

- `online_async_multi_launch.py` publishes maps on `/<robot>/map`.
- If `use_slam:=true` is set in `launch_multi_sim.launch.py`, static TF is `world -> <robot>/map`.
- If `use_slam:=false`, static TF is `world -> <robot>/odom`.

## Teleoperation (Per Robot)

Each robot runs a namespaced `twist_mux`. For manual control, publish to `/<robot>/cmd_vel` (preferred input to mux).

Robot 1:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/robot1/cmd_vel -p stamped:=true
```

Robot 2:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/robot2/cmd_vel -p stamped:=true
```

## Useful Topics

Per robot (`robotN`):

- `/robotN/diff_cont/cmd_vel` (controller input)
- `/robotN/diff_cont/odom`
- `/robotN/scan`
- `/robotN/camera/image_raw`
- `/robotN/camera/camera_info`
- `/robotN/joint_states`
- `/robotN/robot_description`
- `/robotN/map` (when SLAM is active)

Global:

- `/clock`
- `/tf`, `/tf_static`

## Other Launch Files

- `launch/launch_sim.launch.py`: single-robot simulation
- `launch/launch_robot.launch.py`: robot + ros2_control (no Gazebo)
- `launch/online_async_launch.py`: single-robot async SLAM
- `launch/navigation_launch.py`: Nav2 navigation stack launch
- `launch/localization_launch.py`: Nav2 localization launch

## Important Notes

- The repository includes bridge configs for `robot1` and `robot2` (`config/gz_bridge_robot1.yaml`, `config/gz_bridge_robot2.yaml`).
- If `robot_count > 2`, additional bridge config files are needed for each extra robot.
