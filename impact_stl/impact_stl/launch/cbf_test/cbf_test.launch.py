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
            namespace='crackle',
            executable='ff_rate_mpc', # spacecraft_mpc, spacecraft_impact_mpc
            name='crackle_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[{'x0':1.0, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'object_ns':'/pop'}]
        ),

        # Bezier planner
        Node(
            package='push_stl',
            namespace='snap',
            executable='main_planner',
            name='snap_planner',
            output='screen',
            emulate_tty=True,
        ),

    ])
