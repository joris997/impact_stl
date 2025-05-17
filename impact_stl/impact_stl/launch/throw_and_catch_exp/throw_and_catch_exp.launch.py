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
import copy


def generate_launch_description():
    SITL = False
    ld = LaunchDescription()

    ld.add_action(Node(
        package='impact_stl',
            namespace='crackle',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='crackle_mpc',
            # output='screen',
            emulate_tty=True,
            parameters=[{'x0':0.4, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'scenario_name':'throw_and_catch_exp'},
                        {'object_ns':'/snap'}]
    ))
    ld.add_action(Node(
        package='impact_stl',
            namespace='pop',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='pop_mpc',
            # output='screen',
            emulate_tty=True,
            parameters=[{'x0':3.5, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'scenario_name':'throw_and_catch_exp'},
                        {'object_ns':'/snap'}]
    ))
    if not SITL:
        ld.add_action(Node(
            package='impact_stl',
            namespace='snap',
            executable='ff_rate_mpc_velocity_keeping', # spacecraft_mpc, spacecraft_impact_mpc
            name='snap_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[{'x0':1.5, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'object_ns':'/snap'},
                        {'scenario_name':'throw_and_catch_exp'}]
        ))

    # Bezier planner
    ld.add_action(Node(
        package='impact_stl',
        namespace='crackle',
        executable='main_planner',
        name='crackle_planner',
        # output='screen',
        emulate_tty=True,
        parameters=[{'scenario_name':'throw_and_catch_exp'}]
    ))
    ld.add_action(Node(
        package='impact_stl',
        namespace='pop',
        executable='main_planner',
        name='pop_planner',
        # output='screen',
        emulate_tty=True,
        parameters=[{'scenario_name':'throw_and_catch_exp'}]
    ))

    # Impact detector
    if SITL:
        ld.add_action(Node(
            package='impact_stl',
            namespace='crackle',
            executable='impact_detector',
            name='crackle_impact_detector',
            parameters=[{'threshold': 1.0}]
        ))
        ld.add_action(Node(
            package='impact_stl',
            namespace='pop',
            executable='impact_detector',
            name='pop_impact_detector',
            parameters=[{'threshold': 1.0}]
        ))
    else:
        ld.add_action(Node(
            package='impact_stl',
            namespace='snap',
            executable='impact_detector',
            name='snap_impact_detector',
            parameters=[{'threshold': 0.7, 'gz': False}] # 0.5 due to new bumper
        ))
        ld.add_action(Node(
            package='impact_stl',
            namespace='crackle',
            executable='impact_detector',
            name='crackle_impact_detector',
            parameters=[{'threshold': 0.7, 'gz': False}]
        ))
        ld.add_action(Node(
            package='impact_stl',
            namespace='pop',
            executable='impact_detector',
            name='pop_impact_detector',
            parameters=[{'threshold': 0.7, 'gz': False}]
        ))

    # Replanner
    ld.add_action(Node(
        package='impact_stl',
        namespace='crackle',
        executable='replanner',
        name='crackle_replanner',
        # output='screen',
        parameters=[{'object_ns':'/snap'},
                    {'scenario_name':'throw_and_catch_exp'},
                    {'gz': False}]
    ))
    ld.add_action(Node(
        package='impact_stl',
        namespace='pop',
        executable='replanner',
        name='pop_replanner',
        # output='screen',
        parameters=[{'object_ns':'/snap'},
                    {'scenario_name':'throw_and_catch_exp'},
                    {'gz': False}]
    ))


    # VISUALIZATION
    goal_region = ExecuteProcess(
            cmd=[
                'ros2 topic pub --once --qos-reliability reliable /regions/goal_region \
                    visualization_msgs/msg/Marker \
                    "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
                    ns: \'rectangles\', id: 1, type: 1, action: 0,\
                    pose: {position: {x: 2.5, y: 0.0, z: 0.0},\
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
                    scale: {x: 0.5, y: 0.5, z: 0.01},\
                    color: {r: 0.0, g: 1.0, b: 0.0, a: 0.5},\
                    lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
            ],shell=True)
    ld.add_action(goal_region)
   
    return ld