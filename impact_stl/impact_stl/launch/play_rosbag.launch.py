#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # robots = ['snap', 'crackle']
    # robots = ['pop']
    robots = ['snap', 'crackle', 'pop']
    ld = LaunchDescription()
    for i,robot_name in enumerate(robots):
        ld.add_action(Node(
            package='impact_stl',
            namespace=robot_name,
            executable='scenario', 
            name=f'scenario_{i}',
            output='screen',
            emulate_tty=True
        )) 

    # VISUALIZATION
    int_region = ExecuteProcess(
            cmd=[
                'ros2 topic pub --once --qos-reliability reliable /regions/roi_region_1 \
                    visualization_msgs/msg/Marker \
                    "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
                    ns: \'rectangles\', id: 1, type: 1, action: 0,\
                    pose: {position: {x: 1.25, y: 0.0, z: 0.0},\
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
                    pose: {position: {x: 2.75, y: 0.0, z: 0.0},\
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
                    scale: {x: 0.5, y: 0.5, z: 0.01},\
                    color: {r: 0.0, g: 0.0, b: 1.0, a: 0.4},\
                    lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
            ],shell=True)
    ld.add_action(int_region)  
    int_region = ExecuteProcess(
            cmd=[
                'ros2 topic pub --once --qos-reliability reliable /regions/goal_region \
                    visualization_msgs/msg/Marker \
                    "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
                    ns: \'rectangles\', id: 1, type: 1, action: 0,\
                    pose: {position: {x: 2.0, y: 0.0, z: 0.0},\
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
                    scale: {x: 0.5, y: 0.5, z: 0.01},\
                    color: {r: 0.0, g: 1.0, b: 0.0, a: 0.4},\
                    lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
            ],shell=True)
    ld.add_action(int_region)  




    # goal_region = ExecuteProcess(
    #         cmd=[
    #             'ros2 topic pub --once --qos-reliability reliable /regions/goal_region \
    #                 visualization_msgs/msg/Marker \
    #                 "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
    #                 ns: \'rectangles\', id: 1, type: 1, action: 0,\
    #                 pose: {position: {x: 2.5, y: 0.75, z: 0.0},\
    #                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
    #                 scale: {x: 1.0, y: 1.0, z: 0.01},\
    #                 color: {r: 0.0, g: 1.0, b: 0.0, a: 0.4},\
    #                 lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
    #         ],shell=True)
    # ld.add_action(goal_region)
    # int_region = ExecuteProcess(
    #         cmd=[
    #             'ros2 topic pub --once --qos-reliability reliable /regions/roi_region_1 \
    #                 visualization_msgs/msg/Marker \
    #                 "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
    #                 ns: \'rectangles\', id: 1, type: 1, action: 0,\
    #                 pose: {position: {x: 1.25, y: -0.75, z: 0.0},\
    #                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
    #                 scale: {x: 1.0, y: 1.0, z: 0.01},\
    #                 color: {r: 0.0, g: 0.0, b: 1.0, a: 0.4},\
    #                 lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
    #         ],shell=True)
    # ld.add_action(int_region)
    # int_region = ExecuteProcess(
    #         cmd=[
    #             'ros2 topic pub --once --qos-reliability reliable /regions/roi_region_2 \
    #                 visualization_msgs/msg/Marker \
    #                 "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
    #                 ns: \'rectangles\', id: 1, type: 1, action: 0,\
    #                 pose: {position: {x: 2.5, y: -0.75, z: 0.0},\
    #                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
    #                 scale: {x: 1.0, y: 1.0, z: 0.01},\
    #                 color: {r: 0.0, g: 0.0, b: 1.0, a: 0.4},\
    #                 lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
    #         ],shell=True)
    # ld.add_action(int_region)
    # int_region = ExecuteProcess(
    #         cmd=[
    #             'ros2 topic pub --once --qos-reliability reliable /regions/roi_region_3 \
    #                 visualization_msgs/msg/Marker \
    #                 "{header: {frame_id: \'world\', stamp: {sec: 0, nanosec: 0}},\
    #                 ns: \'rectangles\', id: 1, type: 1, action: 0,\
    #                 pose: {position: {x: 1.25, y: 0.30, z: 0.0},\
    #                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},\
    #                 scale: {x: 1.0, y: 1.0, z: 0.01},\
    #                 color: {r: 0.0, g: 0.0, b: 1.0, a: 0.4},\
    #                 lifetime: {sec: 0, nanosec: 0}, frame_locked: false}"'
    #         ],shell=True)
    # ld.add_action(int_region)    

    return ld   
