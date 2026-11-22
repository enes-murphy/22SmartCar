"""Start the M3-2 world and AMCL, with optional map-based Nav2 navigation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    nav_share = Path(get_package_share_directory("smart_car_navigation"))
    slam_share = Path(get_package_share_directory("smart_car_slam"))
    car_share = Path(get_package_share_directory("smart_car_description"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    bringup_share = Path(get_package_share_directory("nav2_bringup"))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gazebo.launch.py")),
        launch_arguments={
            "world": str(slam_share / "worlds" / "scene.world"),
            "verbose": "true",
            "gui": LaunchConfiguration("gui"),
            "params_file": str(car_share / "config" / "gazebo_clock.yaml"),
        }.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                "xacro ", str(car_share / "urdf" / "smart_car.urdf.xacro")
            ]),
            "use_sim_time": True,
        }],
    )
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description", "-entity", "smart_car",
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
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "localization_launch.py")),
        launch_arguments={
            "map": str(nav_share / "maps" / "map.yaml"),
            "params_file": str(nav_share / "config" / "localization.yaml"),
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )
    # Replace the marker with the installed XML path before Nav2 reads YAML.
    nav_params = ReplaceString(
        source_file=str(nav_share / "config" / "navigation.yaml"),
        replacements={
            "__NAV_BT__": str(nav_share / "behavior_trees" / "navigate_ackermann.xml"),
            "__NAV_THROUGH_BT__": str(
                nav_share / "behavior_trees" / "navigate_through_poses_ackermann.xml"
            ),
        },
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "navigation_launch.py")),
        launch_arguments={
            "params_file": nav_params,
            "use_sim_time": "true",
            "autostart": "true",
            "use_composition": "False",
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(nav_share / "config" / "localization.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("navigation", default_value="false"),
        gazebo,
        robot_state_publisher,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_states])),
        RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=[drive])),
        RegisterEventHandler(OnProcessExit(target_action=drive, on_exit=[localization, navigation, rviz])),
        spawn,
    ])
