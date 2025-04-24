#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    """Launch Gazebo with two freeflyers running PX4 communicating over ROS 2."""
    ld = LaunchDescription()

    # Offboard
    ld.add_action(Node(
            package='px4_offboard',
            namespace='crackle',
            executable='visualizer',
            name='visualizer_0'
    )),
    ld.add_action(Node(
            package='px4_offboard',
            namespace='pop',
            executable='visualizer',
            name='visualizer_0'
    )),

    # Rviz
    ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory('push_stl'), 'config.rviz')]]
    ))
#     # Plotjuggler from the juggler_hw_1.xml file (2 spacecrafts)
#     ld.add_action(Node(
#             package='plotjuggler',
#             executable='plotjuggler',
#             name='plotjuggler',
#             arguments=['-l', os.path.join(get_package_share_directory('push_stl'), 'juggler_3.xml')]
#     ))

    # Offboard
    ld.add_action(Node(
            package='px4_offboard',
            namespace='crackle',
            executable='visualizer',
            name='visualizer_0'
    )),
    ld.add_action(Node(
            package='px4_offboard',
            namespace='pop',
            executable='visualizer',
            name='visualizer_0'
    )),

    return ld
