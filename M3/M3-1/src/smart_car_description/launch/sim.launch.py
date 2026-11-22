"""Launch the M3-1 empty world, robot model, and Gazebo spawn process."""

import os
from pathlib import Path
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def gazebo_endpoint(uri):
    """Compare local Gazebo masters by port, including localhost aliases."""
    parsed = urlparse(uri or "http://127.0.0.1:11345")
    host = parsed.hostname or "127.0.0.1"
    if host in ("localhost", "::1"):
        host = "127.0.0.1"
    return host, parsed.port or 11345


def existing_gazebo_servers(proc_root=Path("/proc")):
    """Find servers sharing this launch's Gazebo master, not isolated trials."""
    servers = []
    expected = gazebo_endpoint(os.environ.get("GAZEBO_MASTER_URI"))
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if (entry / "comm").read_text().strip() != "gzserver":
                continue
            args = (entry / "cmdline").read_bytes().split(b"\0")
            world = next((Path(arg.decode(errors="replace")).name for arg in args
                          if arg.endswith(b".world")), "unknown world")
            variables = (entry / "environ").read_bytes().split(b"\0")
            master = next((item.split(b"=", 1)[1].decode(errors="replace")
                           for item in variables if item.startswith(b"GAZEBO_MASTER_URI=")), None)
            if gazebo_endpoint(master) != expected:
                continue
            servers.append((int(entry.name), world))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            # A process can exit while /proc is being read.
            continue
    return sorted(servers)


def generate_launch_description():
    servers = existing_gazebo_servers()
    if servers:
        details = ", ".join(f"PID {pid} ({world})" for pid, world in servers)
        raise RuntimeError(
            "M3-1 launch stopped: Gazebo is already running: " + details + ". "
            "Close the other simulation in its launch terminal (Ctrl+C), "
            "check that 'pgrep -af gzserver' has no output, then retry."
        )

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
