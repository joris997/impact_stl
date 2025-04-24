#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from impact_stl.helpers.qos_profiles import RELIABLE_QOS, NORMAL_QOS

from px4_msgs.msg import VehicleLocalPosition
from my_msgs.msg import StampedBool

class ImpactDetector(Node):
    def __init__(self):
        super().__init__('impact_detector')
        self.threshold = self.declare_parameter('threshold', 1.0).value # 1.0 for sim, 3.0 for hw
        self.gz = self.declare_parameter('gz', True).value

        if self.gz:
            self.local_position_sub = self.create_subscription(
                VehicleLocalPosition,
                'fmu/out/vehicle_local_position_gz',
                self.vehicle_local_position_callback,
                NORMAL_QOS)
        else:
            self.local_position_sub = self.create_subscription(
                VehicleLocalPosition,
                'fmu/out/vehicle_local_position',
                self.vehicle_local_position_callback,
                NORMAL_QOS)
        
        self.publisher_impact = self.create_publisher(StampedBool, 'push_stl/impact_detected', RELIABLE_QOS)

        self.vehicle_acceleration = np.array([0.0, 0.0, 0.0])
        self.vehicle_past_accelerations = np.zeros((3,))

        # get the current time
        self.t_start = Clock().now().nanoseconds/1000
        self.t_wait  = 3
        self.get_logger().info('Created an impact detector')

    def vehicle_local_position_callback(self, msg):
        if (Clock().now().nanoseconds/1000 - self.t_start)/1e6 < self.t_wait:
            return
        # TODO: handle NED->ENU transformation
        self.vehicle_acceleration[0] = msg.ax
        self.vehicle_acceleration[1] = -msg.ay
        self.vehicle_acceleration[2] = -msg.az

        self.vehicle_past_accelerations[:-1] = self.vehicle_past_accelerations[1:]
        self.vehicle_past_accelerations[-1] = np.linalg.norm(self.vehicle_acceleration)

        if np.mean(self.vehicle_past_accelerations) > self.threshold:
            self.get_logger().info('--- IMPACT DETECTED ---')
            msg = StampedBool()
            msg.timestamp = int(Clock().now().nanoseconds / 1000)
            msg.data = True
            self.publisher_impact.publish(msg)
            self.get_logger().info('Impact message published')


def main(args=None):
    rclpy.init(args=args)

    impact_detector = ImpactDetector()

    rclpy.spin(impact_detector)

    impact_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    