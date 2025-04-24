#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from impact_stl.helpers.qos_profiles import RELIABLE_QOS, NORMAL_QOS, GZ_BRIDGE_QOS
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleAngularVelocity, VehicleAttitude

class OdomToVehicleAttitude(Node):
    def __init__(self):
        super().__init__('gz_to_px4_msgs')
        self.topic_name = self.declare_parameter('topic_name', '/model/spacecraft_2d_0/pose').value

        self.odom_sub = self.create_subscription(
            Odometry, 
            f'{self.topic_name}', 
            self.odom_callback, 
            GZ_BRIDGE_QOS)
        
        self.vehicle_attitude_pub = self.create_publisher(
            VehicleAttitude, 
            'fmu/out/vehicle_attitude_gz', 
            NORMAL_QOS)

        self.get_logger().info('GzToPX4Msgs node has been initialized')

    def odom_callback(self, msg):
        # Vehicle Attitude
        va_msg = VehicleAttitude()
        va_msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        va_msg.q[0] = msg.pose.pose.orientation.w
        va_msg.q[1] = msg.pose.pose.orientation.x
        va_msg.q[2] = msg.pose.pose.orientation.y
        va_msg.q[3] = msg.pose.pose.orientation.z

        self.vehicle_attitude_pub.publish(va_msg)


def main(args=None):
    rclpy.init(args=args)
    gzconverter = OdomToVehicleAttitude()
    rclpy.spin(gzconverter)

    gzconverter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    