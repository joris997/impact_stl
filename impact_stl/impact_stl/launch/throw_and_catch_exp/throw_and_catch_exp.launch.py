#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    SITL = True
    ld = LaunchDescription()

    ld.add_action(Node(
        package='push_stl',
            namespace='crackle',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='crackle_mpc',
            # output='screen',
            emulate_tty=True,
            parameters=[{'x0':0.4, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'object_ns':'/snap'}]
    ))
    ld.add_action(Node(
        package='push_stl',
            namespace='pop',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='pop_mpc',
            # output='screen',
            emulate_tty=True,
            parameters=[{'x0':3.5, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'object_ns':'/snap'}]
    ))
    if not SITL:
        ld.add_action(Node(
            package='push_stl',
            namespace='snap',
            executable='ff_rate_mpc_velocity_keeping', # spacecraft_mpc, spacecraft_impact_mpc
            name='snap_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[{'x0':1.2, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0}]
        ))

    # Bezier planner
    ld.add_action(Node(
        package='push_stl',
        namespace='crackle',
        executable='main_planner',
        name='crackle_planner',
        # output='screen',
        emulate_tty=True,
    ))
    ld.add_action(Node(
        package='push_stl',
        namespace='pop',
        executable='main_planner',
        name='pop_planner',
        # output='screen',
        emulate_tty=True,
    ))

    # Impact detector
    if SITL:
        ld.add_action(Node(
            package='push_stl',
            namespace='crackle',
            executable='impact_detector',
            name='crackle_impact_detector',
            parameters=[{'threshold': 1.0}]
        ))
        ld.add_action(Node(
            package='push_stl',
            namespace='pop',
            executable='impact_detector',
            name='pop_impact_detector',
            parameters=[{'threshold': 1.0}]
        ))
    else:
        ld.add_action(Node(
            package='push_stl',
            namespace='crackle',
            executable='impact_detector',
            name='crackle_impact_detector',
            parameters=[{'threshold': 3.0, 'gz': False}]
        ))
        ld.add_action(Node(
            package='push_stl',
            namespace='pop',
            executable='impact_detector',
            name='pop_impact_detector',
            parameters=[{'threshold': 3.0, 'gz': False}]
        ))

    # Replanner
    ld.add_action(Node(
        package='push_stl',
        namespace='crackle',
        executable='replanner',
        name='crackle_replanner',
        # output='screen',
        parameters=[{'object_ns':'/snap'}]
    ))
    ld.add_action(Node(
        package='push_stl',
        namespace='pop',
        executable='replanner',
        name='pop_replanner',
        # output='screen',
        parameters=[{'object_ns':'/snap'}]
    ))
   
    return ld