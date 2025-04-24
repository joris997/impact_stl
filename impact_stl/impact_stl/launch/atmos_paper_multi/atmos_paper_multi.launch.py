#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

# This launch file does a test_run for the ATMOS lab paper
# where a single robot (pop) tracks 4 unique waypoints
# in a sequential fashion from an G_{[0,200]}F_{[a,b]}(x\in A)
# specification

# Requires start_scenario.launch.py to be launced after this one

def generate_launch_description():
    
    return LaunchDescription([
        # MPC controller
        Node(
            package='push_stl',
            namespace='crackle',
            executable='ff_rate_mpc',
            name='spacecraft_mpc_0',
            output='screen',
            emulate_tty=True,
            parameters=[{'x0':0.5, 'y0':0.75, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'object_ns':'/crackle'},
                        {'add_cbf':False}]
        ),
        Node(
            package='push_stl',
            namespace='snap',
            executable='ff_rate_mpc',
            name='spacecraft_mpc_0',
            output='screen',
            emulate_tty=True,
            parameters=[{'x0':0.5, 'y0':-0.75, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'object_ns':'/snap'},
                        {'add_cbf':False}]
        ),

        # Bezier planner
        Node(
            package='push_stl',
            namespace='crackle',
            executable='main_planner',
            name='main_planner_0',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='push_stl',
            namespace='snap',
            executable='main_planner',
            name='main_planner_0',
            output='screen',
            emulate_tty=True,
        ),
    ])
