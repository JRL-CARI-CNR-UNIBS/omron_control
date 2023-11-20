# This Python file uses the following encoding: utf-8

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler, GroupAction
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml

import datetime
import os

def generate_launch_description():

  declared_arguments = []

  declared_arguments.append(
    DeclareLaunchArgument(
        'enable_nav',
        default_value='True',
        description='Whether run nav too'
    )
  )
  enable_nav = LaunchConfiguration('enable_nav')

  log_level = "debug"

  lifecycle_nodes = ['controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower']
  
  nav_base_params = PathJoinSubstitution(
    FindPackageShare('omron_test'),
    'config',
    "nav_params.yaml"
    )
  nav_params = RewrittenYaml(
    source_file=nav_base_params,
    root_key="omron",
    param_rewrites={'autostart': "True"},
    convert_types=True)

  ga = GroupAction(
    condition=IfCondition(enable_nav),
    actions=[PushRosNamespace('omron'),
    Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav_params],
        arguments=['--ros-args', '--log-level', log_level],
        # remappings= [('cmd_vel', '/omron/cmd_vel')]
        ),
    Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav_params],
        arguments=['--ros-args', '--log-level', log_level],
        ),
    Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_params],
        arguments=['--ros-args', '--log-level', log_level],
        ),
    Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav_params],
        arguments=['--ros-args', '--log-level', log_level],
        ),
    Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav_params],
        arguments=['--ros-args', '--log-level', log_level],
        ),
    Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav_params],
        arguments=['--ros-args', '--log-level', log_level],
        ),
    # Node(
    #     package='nav2_velocity_smoother',
    #     executable='velocity_smoother',
    #     name='velocity_smoother',
    #     output='screen',
    #     respawn=use_respawn,
    #     respawn_delay=2.0,
    #     parameters=[configured_params],
    #     arguments=['--ros-args', '--log-level', log_level],
    #     remappings=remappings +
    #             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'autostart': True},
                    {'node_names': lifecycle_nodes}]),
  ])

  to_launch = declared_arguments + [ga]
  return LaunchDescription(to_launch)