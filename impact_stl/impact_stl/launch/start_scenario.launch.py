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
    topics_to_record = []
    for robot in robots:
        topics_to_record.append(f'/{robot}/fmu/out/vehicle_local_position')
        topics_to_record.append(f'/{robot}/odom')
        # reference from the mpc computation
        topics_to_record.append(f'/{robot}/impact_stl/entire_path')
        topics_to_record.append(f'/{robot}/impact_stl/reference_path')
        topics_to_record.append(f'/{robot}/impact_stl/predicted_path')
        # # impact detector
        # topics_to_record.append(f'/{robot}/impact_stl/impact')
        #gazebo ground truth topics
        topics_to_record.append(f'/{robot}/fmu/out/vehicle_angular_velocity_gz')
        topics_to_record.append(f'/{robot}/fmu/out/vehicle_attitude_gz')
        topics_to_record.append(f'/{robot}/fmu/out/vehicle_local_position_gz')
        # control input topics
        topics_to_record.append(f'/{robot}/fmu/in/vehicle_rates_setpoint')

    record_cmd = ['ros2','bag','record']+topics_to_record

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
    ld.add_action(ExecuteProcess(cmd=record_cmd))

    return ld   
