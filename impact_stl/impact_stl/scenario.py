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

class StartScenario(Node):

    def __init__(self):
        super().__init__('scenario_publisher')
        print(f"begin construction of scenario node")       
        self.publisher_compute_plan = self.create_publisher(StampedBool, 'impact_stl/compute_plan', RELIABLE_QOS)
        self.publisher_execute_plan = self.create_publisher(StampedBool, 'impact_stl/execute_plan', RELIABLE_QOS)

        # publish that we should start the computation of the motion plan
        msg = StampedBool()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.data = True
        self.publisher_compute_plan.publish(msg)
        print(f"published compute plan command")

        # wait 5 seconds
        #TODO: fix this such that it knows the completion of the compute_plan
        time.sleep(5)

        # publish that we should start the execution of the motion plan
        msg = StampedBool()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.data = True
        self.publisher_execute_plan.publish(msg)
        print(f"published execute plan command")


def main(args=None):
    rclpy.init(args=args)
    scenario = StartScenario()
    rclpy.spin(scenario)

    scenario.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
