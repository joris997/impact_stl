import rclpy
from rclpy.node import Node

from impact_stl.planners.replanner import RePlanner
from impact_stl.helpers.plot_rvars_hvars import plot_rvars_hvars
import numpy as np
import matplotlib.pyplot as plt

class TestRePlanner(Node):
    def __init__(self):
        super().__init__('test_re_planner')
        replanner = RePlanner(ns='/snap')
        plot_rvars_hvars(replanner.rvars,replanner.hvars,
                         path="/home/none/space_ws/",fn="pre.png")

        replanner.object_local_position = np.array([3.0,0,0])
        #TODO: fix that this works with velocities of the object
        replanner.object_local_velocity = np.array([-0.01,0,0])

        replanner.robot_local_position = np.array([0.0,0,0])
        replanner.robot_local_velocity = np.array([0.05,0,0])

        replanner.solve_replan()

        plot_rvars_hvars(replanner.re_rvars,replanner.re_hvars,
                         path="/home/none/space_ws/",fn="post.png")

def main(args=None):
    rclpy.init(args=args)

    replanner = TestRePlanner()
    rclpy.spin(replanner)
    replanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# [replanner-5] Starting two-body-impact construction
# [replanner-5] xObjectI: [ 2.78750481 -0.30884177]
# [replanner-5] dxObjectI: [ 0.06341981 -0.01140256]
# [replanner-5] dxObjectI_post: [0.1 0. ]
# [replanner-5] theta_init_guess: 3.141592653589793 (180.0 deg)
# [replanner-5] dx_init_guess: [0.05 0.  ]
# [replanner-5] 
# [replanner-5] 
# [replanner-5] Solving took 1.9313621520996094 seconds
# [replanner-5] theta: 3.443761308165096 (197.3129886083113 deg)
# [replanner-5] dx_R_L_post: [-5.91698113e-02  1.87125980e+09]
# [replanner-5] dx_O_L_post: [-0.09546934  0.02975913]
# [replanner-5] dx_R_G_post: [ 5.56870645e+08 -1.78647931e+09]
# [replanner-5] dx_O_G_post: [0.1 0. ]