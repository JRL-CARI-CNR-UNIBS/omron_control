from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch.actions import TimerAction

from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("omron_app"),
                    "urdf",
                    "system.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controller_parameters = PathJoinSubstitution([FindPackageShare("omron_controller"), "config", "parameters.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_parameters],
        output="both"
    )


    spawn_controller = Node(
      package="controller_manager",
      executable="spawner",
      output="screen",
#      parameters=[robot_description, controller_parameters]
      arguments=["omron_velocity_controller"]
    )

    delay_controller_after_manager = TimerAction(
      period = 10.0,
      actions = [spawn_controller]
    )

    nodes = [
        control_node,
        delay_controller_after_manager
    ]

    return LaunchDescription(declared_arguments + nodes)
