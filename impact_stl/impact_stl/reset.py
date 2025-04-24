#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import rclpy
import numpy as np
import time

from rclpy.node import Node
from rclpy.clock import Clock
from impact_stl.helpers.qos_profiles import NORMAL_QOS, RELIABLE_QOS
from my_msgs.msg import StampedBool

class ResetScenario(Node):

    def __init__(self):
        super().__init__('reset_publisher')
        print(f"PUBLISHING RESET MESSAGE")        
        self.publisher_global_reset_plan = self.create_publisher(StampedBool, '/global_reset', RELIABLE_QOS)

        # publish that we should start the computation of the motion plan
        msg = StampedBool()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.data = True
        self.publisher_global_reset_plan.publish(msg)
        print(f"PUBLISHING RESET MESSAGE DONE")



def main(args=None):
    rclpy.init(args=args)
    scenario = ResetScenario()
    rclpy.spin(scenario)

    scenario.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
