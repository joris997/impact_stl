# ROS2 service that considers the position of a robot (snap) and a given target
# and computes a bezier curve trajectory that connects the two points.

import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import casadi as cs
import os

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from my_msgs.msg import BezierCurve, BezierPlan, StampedBool
from my_msgs.msg import VerboseBezierPlan
from my_msgs.srv import SetPlan, SetVerbosePlan
from px4_msgs.msg import VehicleLocalPosition
from impact_stl.helpers.beziers import get_derivative_control_points_gurobi
from ament_index_python.packages import get_package_share_directory
from impact_stl.helpers.read_write_plan import csv_to_plan
from impact_stl.helpers.solve_a_b_plan import solve_a_b_plan

def plan_to_plan_msg(rvars,hvars,idvars,other_names):
    nbzs = len(rvars)
    rvars = [rvar.astype(np.float64) for rvar in rvars]
    hvars = [hvar.astype(np.float64) for hvar in hvars]

    drvars = [get_derivative_control_points_gurobi(rvar) for rvar in rvars]
    dhvars = [get_derivative_control_points_gurobi(hvar) for hvar in hvars]

    plan = VerboseBezierPlan()
    plan.rvar = [BezierCurve() for _ in rvars]
    plan.hvar = [BezierCurve() for _ in hvars]
    plan.drvar = [BezierCurve() for _ in drvars]
    plan.dhvar = [BezierCurve() for _ in dhvars]
    plan.ids = [str() for _ in idvars]
    plan.other_names = [str() for _ in idvars]

    for i in range(nbzs):
        plan.rvar[i].x_cp = rvars[i][0,:].tolist()
        plan.rvar[i].y_cp = rvars[i][1,:].tolist()
        plan.rvar[i].z_cp = rvars[i][2,:].tolist()
        plan.drvar[i].x_cp = drvars[i][0,:].tolist()
        plan.drvar[i].y_cp = drvars[i][1,:].tolist()
        plan.drvar[i].z_cp = drvars[i][2,:].tolist()

        plan.hvar[i].x_cp = hvars[i][0,:].tolist()
        plan.dhvar[i].x_cp = dhvars[i][0,:].tolist()

        plan.ids[i] = idvars[i]
        plan.other_names[i] = other_names[i]
    return plan
    

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetVerbosePlan, 'set_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetVerbosePlan.Request()
    
    def send_request(self, rvars, hvars, idvars, other_names):
        assert(len(rvars) == len(hvars))
        plan = plan_to_plan_msg(rvars,hvars,idvars,other_names)

        self.req.plan = plan
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



class MainPlanner(Node):
    def __init__(self, node):
        super().__init__('simple_planner')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.minimal_client = MinimalClientAsync()
        self.node = node

        self.robot_name = self.get_namespace()
        self.scenario_name = self.declare_parameter('scenario_name','catch_throw').value

        # Subscribers to the state
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        # now we can send the request if the subscriber to "plan" receives a True message
        new_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.compute_plan_sub = self.create_subscription(
            StampedBool,
            'impact_stl/compute_plan',
            self.compute_plan_callback,
            new_qos_profile)
        self.compute_reset_plan_sub = self.create_subscription(
            StampedBool,
            'impact_stl/compute_reset_plan',
            self.compute_reset_plan_callback,
            new_qos_profile)
        
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.init_position = np.array([0.0, 0.0, 0.0])
        self.init_velocity = np.array([0.0, 0.0, 0.0])
        self.get_logger().info('SimplePlanner initialized')        
        
    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz

    def compute_plan_callback(self, msg):
        self.get_logger().info('Computing plan')
        package_share_directory = get_package_share_directory('impact_stl')
        plans_path = os.path.join(package_share_directory)

        self.rvars,self.hvars,self.idvars,self.other_names = csv_to_plan(robot_name=self.robot_name,
                                                                         scenario_name=self.scenario_name,
                                                                         path=plans_path)                      

        # right now we set the init position so we can reset back to it!
        self.init_position = self.vehicle_local_position
        self.get_logger().info('Plan computed')

        if msg.data:
            self.get_logger().info('Sending plan')
            self.minimal_client.send_request(self.rvars, self.hvars, self.idvars, self.other_names)
            self.get_logger().info('Plan received')

    def compute_reset_plan_callback(self,msg):
        self.get_logger().info('Computing reset plan')
        self.rvars,self.hvars,self.idvars = self.compute_reset_plan()
        self.get_logger().info('Reset plan computed')

        if msg.data:
            self.get_logger().info('Sending reset plan')
            self.minimal_client.send_request(self.rvars, self.hvars, self.idvars, self.other_names)
            self.get_logger().info('Reset plan received')

    def compute_reset_plan(self):
        dt = 20
        rvars,hvars,idvars = solve_a_b_plan(self.vehicle_local_position[0:2],
                                            self.vehicle_local_velocity[0:2],
                                            self.init_position[0:2],
                                            self.init_velocity[0:2],dt)
        # add another segment to stay there for a bit as well
        rvars.append(np.zeros((3,4)))
        hvars.append(np.linspace(dt,dt+10,4).reshape(1,4))
        idvars.append('none')
        return rvars,hvars,idvars



def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('main_planner')
    simple_planner = MainPlanner(node)
    rclpy.spin(simple_planner)

    simple_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    