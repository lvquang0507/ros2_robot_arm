from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_package = "robot-arm"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "tx200.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )
    # Spawn robot

    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_tx200",
        arguments=[
            "-entity",
            "tx200",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )
    joint_state_broadcast_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    robot_controllers = ["position_trajectory_controller"]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    delay_JSB_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_spawn_robot,
            on_start=[joint_state_broadcast_node],
        )
    )
    delay_controllers_after_JSB = []
    for controller in robot_controller_spawners:
        delay_controllers_after_JSB += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcast_node,
                    on_exit=[
                        TimerAction(
                            period=3.0,
                            actions=[controller],
                        )
                    ],
                )
            )
        ]
    return LaunchDescription(
        [
            gazebo_node,
            gazebo_spawn_robot,
            robot_state_publisher_node,
            delay_JSB_after_gazebo,
        ]
        + delay_controllers_after_JSB
    )
