"""Launch the M3-2 world, M3-1 robot, controllers, SLAM Toolbox, and optional RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = Path(get_package_share_directory("smart_car_slam"))
    car_share = Path(get_package_share_directory("smart_car_description"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    toolbox_share = Path(get_package_share_directory("slam_toolbox"))

    world = slam_share / "worlds" / "scene.world"
    slam_params = slam_share / "config" / "slam_params.yaml"
    xacro = car_share / "urdf" / "smart_car.urdf.xacro"
    gazebo_clock = car_share / "config" / "gazebo_clock.yaml"
    rviz_config = toolbox_share / "config" / "slam_toolbox_default.rviz"
    use_rviz = LaunchConfiguration("rviz")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gazebo.launch.py")),
        launch_arguments={
            "world": str(world),
            "verbose": "true",
            "params_file": str(gazebo_clock),
        }.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", str(xacro)]),
            "use_sim_time": True,
        }],
    )
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "smart_car",
            "-x", "-5.0", "-y", "-4.0", "-z", "0.03", "-Y", "0.0",
        ],
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
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[str(slam_params), {"use_sim_time": True}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config)],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # Gazebo creates controller_manager only after spawning the robot. Start the
    # two controllers in order, then start SLAM so every required TF already exists.
    start_joint_states = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[joint_states])
    )
    start_drive = RegisterEventHandler(
        OnProcessExit(target_action=joint_states, on_exit=[drive])
    )
    start_mapping = RegisterEventHandler(
        OnProcessExit(target_action=drive, on_exit=[slam, rviz])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Start RViz with the Slam Toolbox display configuration",
        ),
        gazebo,
        robot_state_publisher,
        start_joint_states,
        start_drive,
        start_mapping,
        spawn,
    ])
