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

    # Run the Gazebo simulator and the PX4 SITL simulation. We add a delay
    # to the second robot to ensure they spawn in the same gazebo instance
    # run the px4_1.launch.py script twice
    lf_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('impact_stl'), '/px4.launch.py']),

        launch_arguments={'id': '0', 'pose': '1,1,0', 'name': 'snap', 'delay': '0'}.items()
    )

    lf_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('impact_stl'), '/px4.launch.py']),

        launch_arguments={'id': '1', 'pose': '-7,-1,0', 'name': 'crackle', 'delay': '5'}.items()
    )

    lf_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('impact_stl'), '/px4.launch.py']),

        launch_arguments={'id': '2', 'pose': '-1,0,0', 'name': 'pop', 'delay': '10'}.items()
    )
    ld.add_action(lf_1)
    ld.add_action(lf_2)
    ld.add_action(lf_3)

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


    # Rviz while loading a config file (valid for all three spacecrafts)
    ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory('impact_stl'), 'config.rviz')]]
    ))
    # Plotjuggler from the juggler_2.xml file (2 spacecrafts)
    ld.add_action(Node(
            package='plotjuggler',
            namespace='snap',
            executable='plotjuggler',
            name='plotjuggler',
            arguments=['-l', os.path.join(get_package_share_directory('impact_stl'), 'juggler_sitl_3.xml')]
    ))

    # Launch a Gazebo to ROS bridge such that we can use the ground truth
    # position and velocity estimates in the controller
    # We do this because of the impacts screwing with the EKF2
    # Since we have mo-cap on the platforms, this is a sensible decision
    ld.add_action(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge', # spacecraft_mpc, spacecraft_impact_mpc
            name='bridge_1',
            output='screen',
            arguments=['/model/spacecraft_2d_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                       '/model/spacecraft_2d_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                       '/model/spacecraft_2d_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry']
    ))

    # Launch the gz to px4 converters that take the Odometry message
    # and fill the correct PX4 messages such that we only need to change
    # .../fmu/out/vehicle_local_position to .../fmu/out/vehicle_local_position_gz
    # or vice-versa. Keep care of the namespace and topic_name parameters!!!
    # snap
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_local_position',
            namespace='snap',
            output='screen',
            parameters=[{'topic_name': '/model/spacecraft_2d_0/odometry'}]
    ))
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_attitude',
            namespace='snap',
            output='screen',
            parameters=[{'topic_name': '/model/spacecraft_2d_0/pose'}]
    ))
    # crackle
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_local_position',
            namespace='crackle',
            output='screen',
            parameters=[{'topic_name': '/model/spacecraft_2d_1/odometry'}]
    ))
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_attitude',
            namespace='crackle',
            output='screen',
            parameters=[{'topic_name': '/model/spacecraft_2d_1/pose'}]
    ))
    # pop
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_local_position',
            namespace='pop',
            output='screen',
            parameters=[{'topic_name': '/model/spacecraft_2d_2/odometry'}]
    ))
    ld.add_action(Node(
            package='impact_stl',
            executable='odom_to_vehicle_attitude',
            namespace='pop',
            output='screen',
            parameters=[{'topic_name': '/model/spacecraft_2d_2/pose'}]
    ))

    return ld


