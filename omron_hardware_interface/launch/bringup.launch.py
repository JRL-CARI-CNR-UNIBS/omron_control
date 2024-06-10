from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, NotSubstitution

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
            "rviz",
            default_value="true",
            description="Start RViz2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake",
            default_value="true",
            description="Start as Fake.",
        )
    )

    # Initialize Arguments
    gui =  LaunchConfiguration("rviz")
    fake = LaunchConfiguration("fake")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("omron_description"),
                    "urdf",
                    "system.urdf.xacro",
                ]
            ),
            " ","use_fake_hardware:=","true",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controller_parameters = PathJoinSubstitution([FindPackageShare("omron_controller"), "config", "parameters.yaml"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        condition=IfCondition(fake),
    )

    ###################
    ### Controllers ###
    ###################

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_parameters],
        output="both",
        remappings=[("/odom", "/omron/odom")],
    )

    spawn_controller = Node(
      package="controller_manager",
      executable="spawner",
      output="screen",
#      parameters=[robot_description, controller_parameters]
      condition=IfCondition(NotSubstitution(fake)),
      arguments=["omron_forward_controller"]
    )

    spawn_controller_fake = Node(
      package="controller_manager",
      executable="spawner",
      output="screen",
#      parameters=[robot_description, controller_parameters]
      condition=IfCondition(fake),
      arguments=["omron_fake_position_controller"]
    )

    spawn_state_bcast = Node(
      package="controller_manager",
      executable="spawner",
      output="screen",
#      parameters=[robot_description, controller_parameters]
      arguments=["omron_state_broadcaster"]
    )

    delay_controller_after_manager = TimerAction(
      period = 5.0,
      actions = [spawn_controller, spawn_state_bcast, spawn_controller_fake]
    )

    ###################
    ### Other nodes ###
    ###################

#    map_and_laser_node = Node(
#            package='omron_ros2_agv',
#            executable='omron_ros2_agv_node',
#            namespace='omron',
#            remappings= [('/omron/map', '/map'),('/omron/map_metadata','/map_metadata')],
#            arguments=['--ros-args', '--log-level', 'info'],
#            output='screen')
    map_and_laser_node = Node(
      package='omron_hardware_interface',
      executable='omron_support_nodes',
      arguments=['--ros-args', '--log-level', 'warn'],
      remappings=[('cloud_in', 'omron/cloud_in')],
      parameters=[{"map":"/map",
                   "odom": "/odom",
                   "frame":"omron/base_link"}],
      output='screen',
      condition=IfCondition(NotSubstitution(fake)),
    )

    pcl_to_ls =    Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings={('cloud_in', 'omron/cloud_in'),('scan', 'omron/scan')},
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -2.35,  # -M_PI/2
                'angle_max':  2.35,  # M_PI/2
                'angle_increment': 0.007,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 24.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            arguments=['--ros-args', '--log-level', 'warn'],
            name='pointcloud_to_laserscan',
            condition=IfCondition(NotSubstitution(fake)),
        )

    laser_throttle = Node(
            package='topic_tools',
            executable='throttle',
            namespace='omron',
            parameters=[{
                'input_topic': 'scan',
                'throttle_type': 'messages',
                'msgs_per_sec': 2.0,
                'output_topic': 'scan_rviz'
            }],
            arguments=['messages scan 2 scan_rviz', '--ros-args', '--log-level', 'warn'],
            condition=IfCondition(NotSubstitution(fake)),
            output='screen')

    ## RViz2
    rviz_config = PathJoinSubstitution([FindPackageShare("omron_hardware_interface"), "rviz", "omron.rviz"])
    rviz_node = Node(
      condition=LaunchConfigurationEquals("rviz", "true"),
      package='rviz2',
      executable='rviz2',
      arguments=['--display-config', rviz_config],
    )

    nodes = [
        robot_state_publisher,
        control_node,
        delay_controller_after_manager,
        map_and_laser_node,
        pcl_to_ls,
        laser_throttle,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)
