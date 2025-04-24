#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    return LaunchDescription([
        # MPC controller
        Node(
            package='push_stl',
            namespace='snap',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='spacecraft_mpc_0',
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='push_stl',
        #     namespace='crackle',
        #     executable='spacecraft_mpc', # spacecraft_mpc, spacecraft_impact_mpc
        #     name='spacecraft_mpc_1',
        #     output='screen',
        #     emulate_tty=True,
        # ),
        # Bezier planner
        Node(
            package='push_stl',
            namespace='snap',
            executable='main_planner',
            name='main_planner_0',
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='push_stl',
        #     namespace='crackle',
        #     executable='main_planner',
        #     name='main_planner_1',
        #     output='screen',
        #     emulate_tty=True,
        # ),

        # Impact detector
        Node(
            package='push_stl',
            namespace='snap',
            executable='impact_detector',
            name='impact_detector_0',
            parameters=[{'threshold': 1.0}]
        ),

        # Replanner
        Node(
            package='push_stl',
            namespace='snap',
            executable='replanner',
            name='replanner_0',
            output='screen',
            parameters=[{'object_ns':'/crackle'}]
        ),

    ])
