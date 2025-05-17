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
    SITL = False
    ld = LaunchDescription()

    ld.add_action(Node(
        package='impact_stl',
            namespace='crackle',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='pop_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[{'x0':0.5, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'scenario_name':'pong'},
                        {'object_ns':'/snap'}]
    ))
    ld.add_action(Node(
        package='impact_stl',
            namespace='pop',
            executable='ff_rate_mpc_impact', # spacecraft_mpc, spacecraft_impact_mpc
            name='crackle_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[{'x0':3.5, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'scenario_name':'pong'},
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
            parameters=[{'x0':1.25, 'y0':0.0, 'z0':0.0, 'vx0':0.0, 'vy0':0.0, 'vz0':0.0},
                        {'scenario_name':'pong'},
                        {'object_ns':'/snap'}]
        ))

    # Bezier planner
    ld.add_action(Node(
        package='impact_stl',
        namespace='crackle',
        executable='main_planner',
        name='crackle_planner',
        # output='screen',
        emulate_tty=True,
        parameters=[{'scenario_name':'pong'}]
    ))
    ld.add_action(Node(
        package='impact_stl',
        namespace='pop',
        executable='main_planner',
        name='pop_planner',
        # output='screen',
        emulate_tty=True,
        parameters=[{'scenario_name':'pong'}]
    ))

    # Impact detector
    if SITL:
        ld.add_action(Node(
            package='impact_stl',
            namespace='crackle',
            executable='impact_detector',
            name='crackle_impact_detector',
            parameters=[{'threshold': 0.4, 'gz': True}]
        ))
        ld.add_action(Node(
            package='impact_stl',
            namespace='pop',
            executable='impact_detector',
            name='pop_impact_detector',
            parameters=[{'threshold': 0.4, 'gz': True}]
        ))
    else:
        ld.add_action(Node(
            package='impact_stl',
            namespace='snap',
            executable='impact_detector',
            name='snap_impact_detector',
            parameters=[{'threshold': 0.7, 'gz': False}]
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
        output='screen',
        parameters=[{'object_ns':'/snap'},
                    {'scenario_name':'pong'},
                    {'gz': False}]
    ))
    ld.add_action(Node(
        package='impact_stl',
        namespace='pop',
        executable='replanner',
        name='pop_replanner',
        output='screen',
        parameters=[{'object_ns':'/snap'},
                    {'scenario_name':'pong'},
                    {'gz': False}]
    ))

    # VISUALIZATION
    goal_region = ExecuteProcess(
            cmd=[
                'ros2 topic pub --once --qos-reliability reliable /regions/goal_region \
                    visualization_msgs/msg/Marker \
                    "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
                    ns: \'rectangles\', id: 1, type: 1, action: 0,\
                    pose: {position: {x: 2.5, y: 0.75, z: 0.0},\
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
                    scale: {x: 0.5, y: 0.5, z: 0.01},\
                    color: {r: 0.0, g: 1.0, b: 0.0, a: 0.4},\
                    lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
            ],shell=True)
    ld.add_action(goal_region)

    int_region = ExecuteProcess(
            cmd=[
                'ros2 topic pub --once --qos-reliability reliable /regions/roi_region_1 \
                    visualization_msgs/msg/Marker \
                    "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
                    ns: \'rectangles\', id: 1, type: 1, action: 0,\
                    pose: {position: {x: 1.25, y: -0.75, z: 0.0},\
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
                    scale: {x: 0.5, y: 0.5, z: 0.01},\
                    color: {r: 0.0, g: 0.0, b: 1.0, a: 0.4},\
                    lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
            ],shell=True)
    ld.add_action(int_region)

    int_region = ExecuteProcess(
            cmd=[
                'ros2 topic pub --once --qos-reliability reliable /regions/roi_region_2 \
                    visualization_msgs/msg/Marker \
                    "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
                    ns: \'rectangles\', id: 1, type: 1, action: 0,\
                    pose: {position: {x: 2.5, y: -0.75, z: 0.0},\
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
                    scale: {x: 0.5, y: 0.5, z: 0.01},\
                    color: {r: 0.0, g: 0.0, b: 1.0, a: 0.4},\
                    lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
            ],shell=True)
    ld.add_action(int_region)

    int_region = ExecuteProcess(
            cmd=[
                'ros2 topic pub --once --qos-reliability reliable /regions/roi_region_3 \
                    visualization_msgs/msg/Marker \
                    "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
                    ns: \'rectangles\', id: 1, type: 1, action: 0,\
                    pose: {position: {x: 1.25, y: 0.25, z: 0.0},\
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
                    scale: {x: 0.5, y: 0.5, z: 0.01},\
                    color: {r: 0.0, g: 0.0, b: 1.0, a: 0.4},\
                    lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
            ],shell=True)
    ld.add_action(int_region)    
   
    return ld