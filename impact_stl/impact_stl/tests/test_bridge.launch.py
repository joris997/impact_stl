#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    return LaunchDescription([
        # Launch a Gazebo to ROS bridge such that we can use the ground truth
        # position and velocity estimates in the controller
        # We do this because of the impacts screwing with the EKF2
        # Since we have mo-cap on the platforms, this is a sensible decision
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge', # spacecraft_mpc, spacecraft_impact_mpc
            name='bridge_1',
            output='screen',
            arguments=['/model/spacecraft_2d_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                       '/model/spacecraft_2d_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry']
        ),

        # Launch the gz to px4 converters that take the Odometry message
        # and fill the correct PX4 messages such that we only need to change
        # .../fmu/out/vehicle_local_position to .../fmu/out/vehicle_local_position_gz
        # or vice-versa. Keep care of the namespace and robot_prefix parameters!!!
        Node(
            package='push_stl',
            executable='odom_to_vehicle_local_position',
            namespace='snap',
            output='screen',
            parameters=[{'robot_prefix': '/model/spacecraft_2d_0'}]
        ),
        Node(
            package='push_stl',
            executable='odom_to_vehicle_angular_velocity',
            namespace='snap',
            output='screen',
            parameters=[{'robot_prefix': '/model/spacecraft_2d_0'}]
        ),
        Node(
            package='push_stl',
            executable='odom_to_vehicle_attitude',
            namespace='snap',
            output='screen',
            parameters=[{'robot_prefix': '/model/spacecraft_2d_0'}]
        ),

        Node(
            package='push_stl',
            executable='odom_to_vehicle_local_position',
            namespace='crackle',
            output='screen',
            parameters=[{'robot_prefix': '/model/spacecraft_2d_1'}]
        ),
        Node(
            package='push_stl',
            executable='odom_to_vehicle_angular_velocity',
            namespace='crackle',
            output='screen',
            parameters=[{'robot_prefix': '/model/spacecraft_2d_1'}]
        ),
        Node(
            package='push_stl',
            executable='odom_to_vehicle_attitude',
            namespace='crackle',
            output='screen',
            parameters=[{'robot_prefix': '/model/spacecraft_2d_1'}]
        ),
    ])
