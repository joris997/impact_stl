#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Launch Gazebo with a freeflyer running PX4 communicating over ROS 2."""

    px4_dir = os.getenv("PX4_SPACE_SYSTEMS_DIR")
    if not px4_dir:
        raise RuntimeError("PX4_SPACE_SYSTEMS_DIR is not set. Did you add it to your .bashrc file?")

    return LaunchDescription(
        [
            # We have the first robot always start with id 0
            DeclareLaunchArgument("id", default_value="0"),
            DeclareLaunchArgument("pose", default_value="0,0,0"),
            DeclareLaunchArgument("name", default_value="snap"),
            DeclareLaunchArgument("delay", default_value="0"),
            DeclareLaunchArgument("headless", default_value="1"),
            ExecuteProcess(
                cmd=[
                    "xterm",        # or "gnome-terminal", "konsole", "xterm"
                    "-hold",        # Keep terminal open for debugging
                    "-e",
                    "bash",
                    "-c",
                    "sleep $PX4_DELAY && " +px4_dir+"/build/px4_sitl_default/bin/px4 -i $PX4_INSTANCE",
                ],
                cwd=px4_dir,
                env={**os.environ,
                    "PX4_SIM_AUTOSTART": "4001",
                    "PX4_SIM_SPEED_FACTOR": "1",
                    "PX4_GZ_MODEL_POSE": LaunchConfiguration("pose"),
                    "PX4_INSTANCE": LaunchConfiguration("id"),
                    "PX4_DELAY": LaunchConfiguration("delay"),
                    "PX4_SIM_MODEL": "gz_spacecraft_2d",
                    "PX4_UXRCE_DDS_NS": LaunchConfiguration("name")},
                    # "HEADLESS": LaunchConfiguration("headless")},
                output="screen",
            ),
        ]
    )
