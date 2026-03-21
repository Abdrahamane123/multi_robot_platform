import os
import yaml
import copy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def _load_slam_params(yaml_path):
    """Extract slam_toolbox ros__parameters from a YAML file as a flat dict."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('slam_toolbox', {}).get('ros__parameters', {})


def _slam_nodes(base_params, name, use_sim_time):
    """Create a LifecycleNode + auto-configure/activate events for one robot."""

    params = copy.deepcopy(base_params)
    params['odom_frame'] = f'{name}/odom'
    params['map_frame'] = f'{name}/map'
    params['base_frame'] = f'{name}/base_footprint'
    params['scan_topic'] = f'/{name}/scan'

    node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=name,
        parameters=[params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/map', '/' + name + '/map'),
            ('/map_metadata', '/' + name + '/map_metadata'),
            ('/map_updates', '/' + name + '/map_updates'),
        ],
        output='screen',
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg=f'[LifecycleLaunch] {name}/slam_toolbox activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    return [node, configure, activate]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_count = LaunchConfiguration('robot_count')

    pkg_path = get_package_share_directory('multi_robot_platform')
    base_params = _load_slam_params(
        os.path.join(pkg_path, 'config', 'mapper_params_online_async_robot1.yaml')
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock',
    )
    declare_robot_count = DeclareLaunchArgument(
        'robot_count',
        default_value='2',
        description='Number of robots that run SLAM (robot1..robotN)',
    )

    def launch_setup(context, *args, **kwargs):
        count = int(robot_count.perform(context))
        if count < 1:
            raise ValueError('robot_count must be >= 1')

        actions = []
        for idx in range(1, count + 1):
            name = f'robot{idx}'
            actions.extend(_slam_nodes(base_params, name, use_sim_time))
        return actions

    return LaunchDescription(
        [declare_use_sim_time, declare_robot_count, OpaqueFunction(function=launch_setup)]
    )
