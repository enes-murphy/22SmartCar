"""Launch the M3-1 empty world, robot model, and Gazebo spawn process."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("smart_car_description"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    world = share / "worlds" / "square.world"
    xacro = share / "urdf" / "smart_car.urdf.xacro"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gazebo.launch.py")),
        launch_arguments={
            "world": str(world),
            "verbose": "true",
            "params_file": str(share / "config" / "gazebo_clock.yaml"),
        }.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["xacro ", str(xacro)]), "use_sim_time": True}],
    )
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "smart_car", "-z", "0.03"],
    )
    joint_states = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    drive = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
    )
    # The Gazebo controller manager exists only after the model has spawned.
    start_joint_states = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[joint_states])
    )
    start_drive = RegisterEventHandler(
        OnProcessExit(target_action=joint_states, on_exit=[drive])
    )
    return LaunchDescription([gazebo, robot_state_publisher, start_joint_states, start_drive, spawn])
