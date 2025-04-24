import rclpy
import time
from rclpy.node import Node

from impact_stl.planners.replanner import RePlanner
from push_stl.push_stl.ff_rate_mpc_impact import SpacecraftImpactMPC
from impact_stl.helpers.plot_rvars_hvars import plot_rvars_hvars
from impact_stl.helpers.helpers import VerboseBezierPlan2NumpyArray
import numpy as np
import matplotlib.pyplot as plt

class TestImpactMPC(Node):
    def __init__(self):
        super().__init__('test_impact_mpc')
        print("starting impact-mpc test")
        replanner = RePlanner(ns='/snap')
        plot_rvars_hvars(replanner.rvars,replanner.hvars,
                         path="/home/px4space/space_ws/",fn="pre.png")

        # create mpc object
        mpc = SpacecraftImpactMPC()
        time.sleep(1)
        
        # this should subscribe to replanner.minimalclient so now we can call it
        replanner.minimal_client.send_request(replanner.rvars,replanner.hvars,replanner.idvars)
        mpc.started = True
        mpc.plan = VerboseBezierPlan2NumpyArray()

        mpc.cmdloop_callback()


        # replanner.object_local_position = np.array([3,0,0])
        # replanner.object_local_velocity = np.array([0,0,0])
        # replanner.robot_local_position = np.array([1.0,0,0])
        # replanner.robot_local_velocity = np.array([0.05,0,0])
        # replanner.solve_replan()

        # plot_rvars_hvars(replanner.re_rvars,replanner.re_hvars,
        #                  path="/home/px4space/space_ws/",fn="post.png")

        mpc.cmdloop_callback()

def main(args=None):
    rclpy.init(args=args)

    impact_mpc = TestImpactMPC()
    rclpy.spin(impact_mpc)
    impact_mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()