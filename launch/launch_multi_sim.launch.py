import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, LogInfo, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'multi_robot_platform'
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # ── World ──────────────────────────────────────────────────────────
    default_world = os.path.join(pkg_path, 'worlds', 'empty.world')
    world = LaunchConfiguration('world')
    resolved_world = LaunchConfiguration('resolved_world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load',
    )

    # ── Gazebo (one instance shared by both robots) ────────────────────
    use_slam = LaunchConfiguration('use_slam')
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Set to true when launching SLAM (changes TF: world→map instead of world→odom)',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', resolved_world],
            'on_exit_shutdown': 'true',
        }.items(),
    )
    def _resolve_world_path(context):
        requested_world = world.perform(context)

        # Try user-provided path first.
        if os.path.isfile(requested_world):
            chosen_world = requested_world
            msg = f'[multi_robot_platform] Using world: {chosen_world}'
            return [
                SetLaunchConfiguration('resolved_world', chosen_world),
                LogInfo(msg=msg),
            ]

        # Then try package-local worlds folder with full value and basename.
        candidates = [
            os.path.join(pkg_path, 'worlds', requested_world),
            os.path.join(pkg_path, 'worlds', os.path.basename(requested_world)),
        ]
        for candidate in candidates:
            if os.path.isfile(candidate):
                msg = (
                    '[multi_robot_platform] Provided world not found: '
                    f'{requested_world}. Falling back to: {candidate}'
                )
                return [
                    SetLaunchConfiguration('resolved_world', candidate),
                    LogInfo(msg=msg),
                ]

        # Last resort: built-in default world.
        msg = (
            '[multi_robot_platform] Provided world not found: '
            f'{requested_world}. Falling back to default world: {default_world}'
        )
        return [
            SetLaunchConfiguration('resolved_world', default_world),
            LogInfo(msg=msg),
        ]


    # ── Multi-robot parameters ─────────────────────────────────────────
    robot_count = LaunchConfiguration('robot_count')
    robot_spacing = LaunchConfiguration('robot_spacing')
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='2',
        description='Number of robots to spawn (robot1..robotN)',
    )
    robot_spacing_arg = DeclareLaunchArgument(
        'robot_spacing',
        default_value='18.0',
        description='X spacing in meters between spawned robots',
    )

    # ===================================================================
    #  Helper : build all nodes for one robot
    # ===================================================================
    def robot_nodes(name, x='0.0', y='0.0', z='0.1'):
        """Return a list of nodes for a single robot instance.

        Args:
            name:  unique robot name (used as namespace, model name,
                   frame prefix, and controller-config suffix)
            x,y,z: spawn position in the world
        """

        # ── Robot description (xacro → URDF) ─────────────────────────
        robot_description = Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=true',
            ' sim_mode:=true',
            ' robot_name:=', name,
        ])

        # ── Robot State Publisher (namespaced + frame_prefix) ─────────
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'frame_prefix': name + '/',
            }],
            output='screen',
        )

        # ── Spawn entity in Gazebo ───────────────────────────────────
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/' + name + '/robot_description',
                '-name', name,
                '-x', x,
                '-y', y,
                '-z', z,
            ],
            output='screen',
        )

        # ── Controllers (sequenced: joint_broad first, then diff_cont) ──
        joint_broad_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_broad',
                '-c', '/' + name + '/controller_manager',
                '--controller-manager-timeout', '30',
            ],
        )

        diff_drive_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_cont',
                '-c', '/' + name + '/controller_manager',
                '--controller-manager-timeout', '30',
            ],
        )

        # 1. After spawn completes → start joint_broad
        delayed_joint = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[joint_broad_spawner],
            )
        )

        # 2. After joint_broad finishes → start diff_cont
        delayed_diff = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_broad_spawner,
                on_exit=[diff_drive_spawner],
            )
        )

        delayed_controllers = [delayed_joint, delayed_diff]

        # ── GZ ↔ ROS bridge (sensors) ───────────────────────────────
        bridge_cfg = os.path.join(
            pkg_path, 'config', f'gz_bridge_{name}.yaml',
        )
        gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args', '-p', f'config_file:={bridge_cfg}',
            ],
        )

        # Image bridge (camera)
        gz_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=[f'/{name}/camera/image_raw'],
        )

        # ── Twist-mux (one per robot, remapping to its namespace) ────
        twist_mux_params = os.path.join(
            pkg_path, 'config', 'twist_mux.yaml',
        )
        twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            namespace=name,
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[
                ('/cmd_vel_out', '/' + name + '/diff_cont/cmd_vel'),
            ],
        )

        # ── Static TF: world → <name>/map  OR  world → <name>/odom ──
        #    With SLAM:    world → map → odom → base_link
        #    Without SLAM: world → odom → base_link
        static_tf_slam = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', x, '--y', y, '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', name + '/map',
            ],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_slam),
        )

        static_tf_no_slam = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', x, '--y', y, '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', name + '/odom',
            ],
            parameters=[{'use_sim_time': True}],
            condition=UnlessCondition(use_slam),
        )

        return [
            rsp,
            spawn,
            *delayed_controllers,
            gz_bridge,
            gz_image_bridge,
            twist_mux,
            static_tf_slam,
            static_tf_no_slam,
        ]

    def _write_dynamic_rviz_config(count):
        """Create a temporary RViz config that scales displays to robot1..robotN."""
        colors = [
            '255; 0; 0',
            '0; 0; 255',
            '0; 180; 0',
            '255; 140; 0',
            '180; 0; 180',
            '0; 180; 180',
        ]

        displays = [
            {
                'Class': 'rviz_default_plugins/Grid',
                'Name': 'Grid',
                'Enabled': True,
                'Value': True,
                'Cell Size': 1,
                'Line Style': {'Line Width': 0.03, 'Value': 'Lines'},
                'Normal Cell Count': 0,
                'Plane': 'XY',
                'Plane Cell Count': 20,
                'Reference Frame': '<Fixed Frame>',
            }
        ]

        for idx in range(1, count + 1):
            name = f'robot{idx}'
            color = colors[(idx - 1) % len(colors)]

            displays.append({
                'Class': 'rviz_default_plugins/RobotModel',
                'Name': f'Robot{idx} Model',
                'Enabled': True,
                'Description Source': 'Topic',
                'Description Topic': {
                    'Value': f'/{name}/robot_description',
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                },
                'TF Prefix': name,
                'Update Interval': 0,
                'Visual Enabled': True,
                'Alpha': 1,
            })

            displays.append({
                'Class': 'rviz_default_plugins/LaserScan',
                'Name': f'Robot{idx} Scan',
                'Enabled': True,
                'Topic': {
                    'Value': f'/{name}/scan',
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort',
                },
                'Size (m)': 0.03,
                'Color': color,
            })

            displays.append({
                'Class': 'rviz_default_plugins/Odometry',
                'Name': f'Robot{idx} Odom',
                'Enabled': False,
                'Topic': {
                    'Value': f'/{name}/diff_cont/odom',
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                },
                'Keep': 100,
            })

            displays.append({
                'Class': 'rviz_default_plugins/Map',
                'Name': f'Robot{idx} Map',
                'Enabled': True,
                'Topic': {
                    'Value': f'/{name}/map',
                    'Depth': 5,
                    'Durability Policy': 'Transient Local',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                },
                'Alpha': 0.7,
                'Color Scheme': 'map',
                'Draw Behind': True,
            })

        displays.append({
            'Class': 'rviz_default_plugins/TF',
            'Name': 'TF',
            'Enabled': True,
            'Show Names': False,
            'Show Arrows': True,
            'Show Axes': True,
            'Marker Scale': 0.3,
        })

        config = {
            'Panels': [
                {'Class': 'rviz_common/Displays', 'Name': 'Displays'},
                {'Class': 'rviz_common/Views', 'Name': 'Views'},
            ],
            'Visualization Manager': {
                'Class': '',
                'Displays': displays,
                'Global Options': {
                    'Background Color': '48; 48; 48',
                    'Fixed Frame': 'world',
                    'Frame Rate': 30,
                },
                'Name': 'root',
                'Tools': [
                    {'Class': 'rviz_default_plugins/Interact', 'Hide Inactive Objects': True},
                    {'Class': 'rviz_default_plugins/MoveCamera'},
                    {'Class': 'rviz_default_plugins/FocusCamera'},
                ],
                'Value': True,
                'Views': {
                    'Current': {
                        'Class': 'rviz_default_plugins/Orbit',
                        'Distance': 5,
                        'Name': 'Current View',
                        'Near Clip Distance': 0.01,
                        'Pitch': 0.5,
                        'Target Frame': '<Fixed Frame>',
                        'Value': 'Orbit (rviz_default_plugins)',
                        'Yaw': 0.5,
                    }
                },
            },
            'Window Geometry': {
                'Height': 800,
                'Width': 1200,
                'Displays': {'collapsed': False},
                'Views': {'collapsed': True},
            },
        }

        fd, path = tempfile.mkstemp(prefix='multi_robot_', suffix='.rviz')
        os.close(fd)
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(config, f, sort_keys=False)
        return path

    def launch_setup(context, *args, **kwargs):
        """Build all robot actions dynamically from robot_count."""
        count = int(robot_count.perform(context))
        spacing = float(robot_spacing.perform(context))

        if count < 1:
            raise ValueError('robot_count must be >= 1')

        actions = [gazebo]
        for idx in range(1, count + 1):
            name = f'robot{idx}'
            x = str((idx - 1) * spacing)
            actions.extend(robot_nodes(name, x=x, y='0.0'))

        rviz_config = _write_dynamic_rviz_config(count)
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
        actions.append(rviz)
        return actions

    # ── Assemble the launch description ───────────────────────────────
    return LaunchDescription([
        world_arg,
        use_slam_arg,
        robot_count_arg,
        robot_spacing_arg,
        OpaqueFunction(function=_resolve_world_path),
        OpaqueFunction(function=launch_setup),
    ])
