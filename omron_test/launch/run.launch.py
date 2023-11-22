# This Python file uses the following encoding: utf-8

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import datetime
import os

def generate_launch_description():

  save_bags = LaunchConfiguration('save_bags')

  save_bags_arg = DeclareLaunchArgument(
     'save_bags',
     default_value="True",
  )

  config = PathJoinSubstitution([FindPackageShare("omron_test"), "config", "test_parameters.yaml"])
#  bagname = f"motion_{datetime.datetime.today().year}_{datetime.datetime.today().month}_{datetime.datetime.today().day}_{datetime.datetime.today().hour}_{datetime.datetime.today().minute}_{datetime.datetime.today().second}"
  bagpath = os.path.join(os.environ["HOME"],"projects","bags")

  motion_node = Node(
    package="omron_test",
    executable="omron_test_node",
    output="screen",
    parameters=[config]
  )

  rosbag_record = Node(
    package="omron_test",
    executable="omron_test_bag_node",
    name="omron_test_bag_node",
    output="screen",
    parameters=[config, {"bag_path": bagpath}]
  )

  # rosbag_record = ExecuteProcess(
  #     condition=IfCondition(
  #       PythonExpression([save_bags])
  #     ),
  #     cmd=['ros2', 'bag', 'record', '-a', '-o', bagpath],
  #     output='log',
  # )

  close_bag = RegisterEventHandler(
    OnProcessExit(
      target_action=motion_node,
      on_exit=[EmitEvent(event=Shutdown(reason="Motion terminated"))]
    )
  )

  ################
  ## Navigation ##
  ######################################
  ## For return home after every move ##
  ######################################
  launch_include = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          PathJoinSubstitution([
              FindPackageShare('omron_test'),
              'launch',
              'nav.launch.py']))
  )

  to_launch = [save_bags_arg,
               motion_node,
               rosbag_record,
               close_bag,
               launch_include]

  return LaunchDescription(to_launch)
