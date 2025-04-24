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

class OdomToVehicleAngularVelocity(Node):
    def __init__(self):
        super().__init__('gz_to_px4_msgs')
        self.topic_name = self.declare_parameter('topic_name', '/model/spacecraft_2d_0').value

        self.odom_sub = self.create_subscription(
            Odometry, 
            f'{self.topic_name}', 
            self.odom_callback, 
            GZ_BRIDGE_QOS)
        
        self.vehicle_angular_velocity_pub = self.create_publisher(
            VehicleAngularVelocity, 
            'fmu/out/vehicle_angular_velocity_gz', 
            NORMAL_QOS)
        
        self.vx_prev = 0.0
        self.vy_prev = 0.0
        self.vz_prev = 0.0
        self.t_prev  = 0.0
        self.get_logger().info('GzToPX4Msgs node has been initialized')

    def odom_callback(self, msg):
        # Vehicle Angular Velocity
        vav_msg = VehicleAngularVelocity()
        vav_msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        vav_msg.xyz[0] = msg.twist.twist.angular.x
        vav_msg.xyz[1] = msg.twist.twist.angular.y
        vav_msg.xyz[2] = msg.twist.twist.angular.z

        self.vehicle_angular_velocity_pub.publish(vav_msg)

def main(args=None):
    rclpy.init(args=args)
    gzconverter = OdomToVehicleAngularVelocity()
    rclpy.spin(gzconverter)

    gzconverter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    