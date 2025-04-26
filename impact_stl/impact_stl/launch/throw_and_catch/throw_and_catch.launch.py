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
            package='impact_stl',
            namespace='snap',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='snap_mpc',
            # output='screen',
            emulate_tty=True,
            parameters=[{'x0':1.0, 'y0':1.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'scenario_name':'throw_and_catch'},
                        {'object_ns':'/pop'}]
        ),
        Node(
            package='impact_stl',
            namespace='crackle',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='crackle_mpc',
            # output='screen',
            emulate_tty=True,
            parameters=[{'x0':1.0, 'y0':9.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'scenario_name':'throw_and_catch'},
                        {'object_ns':'/pop'}]
        ),
        # # run the velocity keeping MPC for the object (pop)?
        # Node(
        #     package='impact_stl',
        #     namespace='pop',
        #     executable='ff_rate_mpc_velocity_keeping', # spacecraft_mpc, spacecraft_impact_mpc
        #     name='pop_mpc',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[{'x0':2.0, 'y0':5.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0}]
        # ),

        # Bezier planner
        Node(
            package='impact_stl',
            namespace='snap',
            executable='main_planner',
            name='snap_planner',
            # output='screen',
            emulate_tty=True,
            parameters=[{'scenario_name':'throw_and_catch'}]
        ),
        Node(
            package='impact_stl',
            namespace='crackle',
            executable='main_planner',
            name='crackle_planner',
            # output='screen',
            emulate_tty=True,
            parameters=[{'scenario_name':'throw_and_catch'}]
        ),

        # Impact detector
        Node(
            package='impact_stl',
            namespace='snap',
            executable='impact_detector',
            name='snap_impact_detector',
            parameters=[{'threshold': 0.75}]
        ),
        Node(
            package='impact_stl',
            namespace='crackle',
            executable='impact_detector',
            name='crackle_impact_detector',
            parameters=[{'threshold': 0.75}]
        ),
        Node(
            package='impact_stl',
            namespace='pop',
            executable='impact_detector',
            name='pop_impact_detector',
            parameters=[{'threshold': 0.75}]
        ),

        # Replanner
        Node(
            package='impact_stl',
            namespace='snap',
            executable='replanner',
            name='snap_replanner',
            # output='screen',
            parameters=[{'object_ns':'/pop'},
                        {'scenario_name':'throw_and_catch'}]
        ),
        # Replanner
        Node(
            package='impact_stl',
            namespace='crackle',
            executable='replanner',
            name='crackle_replanner',
            # output='screen',
            parameters=[{'object_ns':'/pop'},
                        {'scenario_name':'throw_and_catch'}]
        ),

    ])
