#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    """Launch Gazebo with two freeflyers running PX4 communicating over ROS 2."""
    ld = LaunchDescription()

    # Visualizer nodes which subscribe to the PX4 topics and converts them to sensible
    # topics for rviz
    ld.add_action(Node(
            package='px4_offboard',
            namespace='snap',
            executable='visualizer',
            name='visualizer_0'
    )),
    ld.add_action(Node(
            package='px4_offboard',
            namespace='crackle',
            executable='visualizer',
            name='visualizer_1',
    )),
    ld.add_action(Node(
            package='px4_offboard',
            namespace='pop',
            executable='visualizer',
            name='visualizer_1',
    )),

    # camera 
    ld.add_action(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_inertial',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'inertial']
        )),
    ld.add_action(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_world_to_camera',
                # arguments=['0', '0', '2.5', '0', '0.4349655', '0', '0.9', 'world', 'camera_link'] # camera 1
                arguments=['2', '1.9', '2.3', '0.3010647', '0.3013046', '-0.6395013', '0.6400107', 'world', 'camera_link'] # camera 2
        )),


    # Rviz while loading a config file (valid for all three spacecrafts)
    ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory('impact_stl'), 'config.rviz')]]
    ))
#     # Plotjuggler from the juggler_2.xml file (2 spacecrafts)
#     ld.add_action(Node(
#             package='plotjuggler',
#             namespace='snap',
#             executable='plotjuggler',
#             name='plotjuggler',
#             arguments=['-l', os.path.join(get_package_share_directory('impact_stl'), 'juggler_sitl_3.xml')]
#     ))

    # Launch the gz to px4 converters that take the Odometry message
    # and fill the correct PX4 messages such that we only need to change
    # .../fmu/out/vehicle_local_position to .../fmu/out/vehicle_local_position_gz
    # or vice-versa. Keep care of the namespace and robot_prefix parameters!!!
    # snap
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_local_position',
            namespace='snap',
            output='screen',
            parameters=[{'topic_name': '/snap/odom'}]
    ))
    # crackle
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_local_position',
            namespace='crackle',
            output='screen',
            parameters=[{'topic_name': '/crackle/odom'}]
    ))
    # pop
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_local_position',
            namespace='pop',
            output='screen',
            parameters=[{'topic_name': '/pop/odom'}]
    ))

    return ld


