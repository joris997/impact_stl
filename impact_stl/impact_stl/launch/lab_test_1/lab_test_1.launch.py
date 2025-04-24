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
            namespace='pop',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='crackle_mpc',
            # output='screen',
            emulate_tty=True,
            parameters=[{'x0':0.5, 'y0':0.5, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0,
                          'object_ns':'/crackle'}]
        ),
        # run the velocity keeping MPC for the object (crackle)?
        Node(
            package='push_stl',
            namespace='crackle',
            executable='ff_rate_mpc_velocity_keeping', # spacecraft_mpc, spacecraft_impact_mpc
            name='pop_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[{'x0':2.5, 'y0':-0.5, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0}]
        ),

        # Bezier planner
        Node(
            package='push_stl',
            namespace='pop',
            executable='main_planner',
            name='pop_planner',
            # output='screen',
            emulate_tty=True,
        ),

        # Impact detector
        Node(
            package='push_stl',
            namespace='crackle',
            executable='impact_detector',
            name='crackle_impact_detector',
            parameters=[{'threshold': 3.0, 'gz': False}]
        ),
        Node(
            package='push_stl',
            namespace='pop',
            executable='impact_detector',
            name='pop_impact_detector',
            parameters=[{'threshold': 3.0, 'gz': False}]
        ),

        # Replanner
        Node(
            package='push_stl',
            namespace='pop',
            executable='replanner',
            name='snap_replanner',
            # output='screen',
            parameters=[{'object_ns':'/crackle', 'gz': False}]
        ),

    ])
