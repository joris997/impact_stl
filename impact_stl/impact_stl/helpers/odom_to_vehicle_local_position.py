#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from impact_stl.helpers.qos_profiles import RELIABLE_QOS, NORMAL_QOS, GZ_BRIDGE_QOS
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition

class OdomToVehicleLocalPosition(Node):
    def __init__(self):
        super().__init__('gz_to_px4_msgs')
        self.topic_name = self.declare_parameter('topic_name', '/model/spacecraft_2d_0').value

        self.odom_sub = self.create_subscription(
            Odometry, 
            f'{self.topic_name}', 
            self.odom_callback, 
            GZ_BRIDGE_QOS)
        
        self.vehicle_local_position_pub = self.create_publisher(
            VehicleLocalPosition, 
            'fmu/out/vehicle_local_position_gz', 
            NORMAL_QOS)
        
        self.vx_prev = 0.0
        self.vy_prev = 0.0
        self.vz_prev = 0.0
        self.t_prev  = 0.0
        self.get_logger().info('GzToPX4Msgs node has been initialized')

    def odom_callback(self, msg):
        # Vehicle Local Position
        vlp_msg = VehicleLocalPosition()
        vlp_msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        vlp_msg.x = msg.pose.pose.position.y
        vlp_msg.y = msg.pose.pose.position.x
        vlp_msg.z = msg.pose.pose.position.z
        vlp_msg.vx = msg.twist.twist.linear.y
        vlp_msg.vy = msg.twist.twist.linear.x
        vlp_msg.vz = msg.twist.twist.linear.z
        dt = self.get_clock().now().nanoseconds/1e9 - self.t_prev/1e9
        vlp_msg.ax = (msg.twist.twist.linear.y - self.vx_prev) / dt
        vlp_msg.ay = (msg.twist.twist.linear.x - self.vy_prev) / dt
        vlp_msg.az = (msg.twist.twist.linear.z - self.vz_prev) / dt

        self.vehicle_local_position_pub.publish(vlp_msg)\

        self.vx_prev = msg.twist.twist.linear.y
        self.vy_prev = msg.twist.twist.linear.x
        self.vz_prev = msg.twist.twist.linear.z
        self.t_prev = self.get_clock().now().nanoseconds

def main(args=None):
    rclpy.init(args=args)
    gzconverter = OdomToVehicleLocalPosition()
    rclpy.spin(gzconverter)

    gzconverter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    