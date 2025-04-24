#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import rclpy
import numpy as np
import time
import os

from impact_stl.helpers.read_write_plan import csv_to_plan

from rclpy.node import Node
from rclpy.clock import Clock
from impact_stl.helpers.qos_profiles import NORMAL_QOS, RELIABLE_QOS

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleRatesSetpoint

from my_msgs.srv import SetPlan, SetVerbosePlan
from my_msgs.msg import StampedBool, VerboseBezierPlan

from ament_index_python.packages import get_package_share_directory

from impact_stl.models.spacecraft_rate_model import SpacecraftRateModel
from impact_stl.planners.main_planner import plan_to_plan_msg
# from push_stl.controller.rate_mpc import SpacecraftRateMPC
from impact_stl.controllers.rate_mpc import SpacecraftRateMPC
from impact_stl.controllers.rate_impact_mpc import SpacecraftRateImpactMPC
from impact_stl.helpers.helpers import vector2PoseMsg, BezierCurve2NumpyArray, \
                            BezierPlan2NumpyArray, interpolate_bezier, VerboseBezierPlan2NumpyArray,\
                            Quaternion2Euler, Euler2Quaternion
from planner.utilities.beziers import get_derivative_control_points_gurobi, value_bezier, eval_t


class SpacecraftCleanMPC(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Creating SpacecraftImpactMPC node')
        # the object is an actual robot, so it has a namespace that we need 
        # for properly timing the replanning
        self.robot_name = self.get_namespace()
        self.object_ns = self.declare_parameter('object_ns', '/pop').value

        # get initial state from passed parameters
        self.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            np.sqrt(2)/2, 0.0, 0.0, -np.sqrt(2)/2]).reshape(10, 1)
        self.x0[0] = self.declare_parameter('x0', 0.0).value
        self.x0[1] = self.declare_parameter('y0', 0.0).value
        self.x0[2] = self.declare_parameter('z0', 0.0).value
        self.x0[3] = self.declare_parameter('vx0', 0.0).value
        self.x0[4] = self.declare_parameter('vy0', 0.0).value
        self.x0[5] = self.declare_parameter('vz0', 0.0).value

        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            NORMAL_QOS)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            'fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            NORMAL_QOS)
        self.angular_vel_sub = self.create_subscription(
            VehicleAngularVelocity,
            'fmu/out/vehicle_angular_velocity',
            self.vehicle_angular_velocity_callback,
            NORMAL_QOS)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            NORMAL_QOS)
        
        # Bezier planner stuff
        self.execute_plan_sub = self.create_subscription(
            StampedBool,
            'push_stl/execute_plan',
            self.execute_plan_callback,
            RELIABLE_QOS)
        
        # Impact detector
        self.impact_detected_sub = self.create_subscription(
            StampedBool,
            'push_stl/impact_detected',
            self.impact_detected_callback,
            RELIABLE_QOS)
        self.impacted = False
        self.impact_time = 0

        # Reset to origin listener
        self.reset_sub = self.create_subscription(
            StampedBool,
            'push_stl/reset',
            self.reset_callback,
            RELIABLE_QOS)
        
        # Global reset listener
        self.global_reset_sub = self.create_subscription(
            StampedBool,
            '/global_reset',
            self.global_reset_callback,
            RELIABLE_QOS)
        
        # controller heuristics
        self.post_impact_backup_duration = 0.5  # how long we turn the contoller off after an impact
        self.t_object_coming = np.inf           # time when the object is coming to our impact point
        self.has_impacted = False               # flag to indicate that we have impacted the object

        # get the plan of the object from the csv file
        # this plan also never changes, so we can load it and use it forever :)
        package_share_directory = get_package_share_directory('push_stl')
        plans_path = os.path.join(package_share_directory)
        try:
            rvar,hvar,ids,other_names = csv_to_plan(self.object_ns,path=plans_path)
            self.plan_object = VerboseBezierPlan2NumpyArray(plan_to_plan_msg(rvar,hvar,ids,other_names))
            # print some info
            self.get_logger().info(f"Number of bezier segments: {len(self.plan_object['rvar'])}")
            self.get_logger().info(f"Number of control points: {self.plan_object['rvar'][0].shape[1]}")
            self.get_logger().info(f"Segment ids: {self.plan_object['ids']}")
        except Exception as e:
            print(f"Could not find plan for object {self.object_ns}")
            print(f"Error: {e}")
            self.plan_object = None
        
        # service for obtaining an initial and an updated motion plan!
        # because this one can change, we can't just read the plan once and save it
        self.set_plan_srv = self.create_service(SetVerbosePlan, 'set_plan', self.add_set_plan_callback)
        self.start_time = 0     # Time when the plan starts, used to interpolate the Bezier curves to get the plan
        self.started = False    # Flag to start the plan, based on start_plan bool topic
        self.plan = None

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', NORMAL_QOS)
        self.publisher_rates_setpoint = self.create_publisher(VehicleRatesSetpoint, 'fmu/in/vehicle_rates_setpoint', NORMAL_QOS)
        self.predicted_path_pub = self.create_publisher(Path, 'push_stl/predicted_path', 10)
        self.reference_path_pub = self.create_publisher(Path, "push_stl/reference_path", 10)
        self.entire_path_pub = self.create_publisher(Path, "push_stl/entire_path", 10)
        self.publisher_recompute_local_plan = self.create_publisher(StampedBool, 'push_stl/recompute_local_plan', RELIABLE_QOS)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # Velocity keeping stuff
        stack_size = 30
        self.xplus_stack = np.zeros((10, stack_size))
        self.plan = {}
        self.plan['rvar'] = [np.linspace(self.x0.reshape((10,)), self.x0.reshape((10,)), 2).T]
        self.plan['hvar'] = [np.linspace(0,100,2).reshape((1,2))]
        self.plan['drvar'] = [get_derivative_control_points_gurobi(self.plan['rvar'][0],1)]
        self.plan['dhvar'] = [get_derivative_control_points_gurobi(self.plan['hvar'][0],1)]
        self.plan['ids'] = ['none']
        self.plan['other_names'] = ['none']

        # Create Spacecraft and controller objects
        self.model = SpacecraftRateModel()
        self.mpc = SpacecraftRateMPC(self.model,Tf=1.0,N=10)
        self.initial_guess = {'X': None, 'U': None}

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])

    def global_reset_callback(self, msg):
        self.get_logger().info('Global reset received')
        self.started = False
        self.impacted = False
        self.has_impacted = False
        self.t_object_coming = np.inf

    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz
        self.xplus_stack[:,0:-1] = self.xplus_stack[:,1:]
        self.xplus_stack[:,-1] = np.array([msg.x, -msg.y, -msg.z, msg.vx, -msg.vy, -msg.vz,
                                           1.0, 0.0, 0.0, 0.0]).reshape(10,)

    def vehicle_angular_velocity_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_angular_velocity[0] = msg.xyz[0]
        self.vehicle_angular_velocity[1] = -msg.xyz[1]
        self.vehicle_angular_velocity[2] = -msg.xyz[2]

    def vehicle_status_callback(self, msg):
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def reset_callback(self, msg):
        self.get_logger().info('Resetting object to x0')
        if msg.data:
            self.impacted = False
            self.has_impacted = False
            self.impact_time = 0

    def impact_detected_callback(self, msg):
        self.get_logger().info('Impact received')
        if msg.data:
            if self.impacted is not True:
                self.impact_time = Clock().now().nanoseconds / 1000
            self.impacted = True
        # self.impact_time = Clock().now().nanoseconds / 1000

    def publish_rate_setpoint(self, u_pred):
        thrust_rates = u_pred[:, 0]
        thrust_command = thrust_rates[0:3]
        rates_setpoint_msg = VehicleRatesSetpoint()
        rates_setpoint_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        rates_setpoint_msg.roll  = float(thrust_rates[3])
        rates_setpoint_msg.pitch = -float(thrust_rates[4])
        rates_setpoint_msg.yaw   = -float(thrust_rates[5])
        rates_setpoint_msg.thrust_body[0] = float(thrust_command[0])
        rates_setpoint_msg.thrust_body[1] = -float(thrust_command[1])
        rates_setpoint_msg.thrust_body[2] = -float(thrust_command[2])
        self.publisher_rates_setpoint.publish(rates_setpoint_msg)

    def get_setpoints(self):
        t = (Clock().now().nanoseconds / 1000 - self.start_time) / 1e6
        setpoints = []
        weights = {'Q': None, 'Q_e': None, 'R': None}

        setpoint = self.x0
        for i in range(self.mpc.N+1):
            setpoints.append(setpoint)
        
        return setpoints, weights
    
    def get_setpoints_velocity_keeping(self):
        t = (Clock().now().nanoseconds / 1000 - self.start_time) / 1e6        
        # eval the bezier curves
        setpoints = []
        # print(f"rvar: {self.plan['rvar']}")
        # print(f"drvar: {self.plan['drvar']}")
        # print(f"dhvar: {self.plan['dhvar']}")

        for i in range(self.mpc.N+1):
            tmpc = t + i*self.mpc.dt
            idx, s = eval_t(self.plan['hvar'], tmpc)
            q = value_bezier(self.plan['rvar'][idx],s)
            dq = value_bezier(self.plan['drvar'][idx],s)/value_bezier(self.plan['dhvar'][idx],s)
            # print(f"q: {q}")
            # print(f"dq: {dq}")
            setpoint = np.array([q[0], q[1], q[2],
                                 dq[0], dq[1], dq[2],
                                 self.x0[6,0], self.x0[7,0], self.x0[8,0], self.x0[9,0]]).reshape(10, 1)
            setpoints.append(setpoint)
        # print(f"s: {s}")

        return setpoints

    def cmdloop_callback(self):
        t0 = time.time()
        t = (Clock().now().nanoseconds / 1000 - self.start_time) / 1e6
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.direct_actuator = False
        offboard_msg.body_rate = True   # rate control
        self.publisher_offboard_mode.publish(offboard_msg)

        x0 = np.array([self.vehicle_local_position[0], self.vehicle_local_position[1], self.vehicle_local_position[2],
                       self.vehicle_local_velocity[0], self.vehicle_local_velocity[1], self.vehicle_local_velocity[2],
                       self.vehicle_attitude[0], self.vehicle_attitude[1], self.vehicle_attitude[2], self.vehicle_attitude[3]]).reshape(10, 1)
        # print(f"x0: {x0}")

        # get the reference states and corresponding times in the horizon
        setpoints, weights = self.get_setpoints()
        if self.has_impacted:
            setpoints = self.get_setpoints_velocity_keeping()
        
        # solve the mpc
        # now the initial guess has to be x_pred again, but check the size!
        if self.initial_guess['X'] is not None:
            self.initial_guess['X'] = self.initial_guess['X'][:,1::] if self.initial_guess['X'].shape[1] > self.mpc.N+1 else self.initial_guess['X']    
            self.initial_guess['U'] = self.initial_guess['U'][:,1::] if self.initial_guess['U'].shape[1] > self.mpc.N else self.initial_guess['U']

        x_pred, u_pred = self.mpc.solve(x0,setpoints,
                                        weights=weights,
                                        initial_guess=self.initial_guess,
                                        verbose=False)
            
        # if we just impacted the object, turn off the controller to prevent further pushing
        dt_post_impact = (Clock().now().nanoseconds / 1000 - self.impact_time)/1e6
        # print(f"dt_post_impact: {dt_post_impact}")
        if self.impacted and dt_post_impact < self.post_impact_backup_duration:
            self.get_logger().info('Impact detected, stopping controller')
            u_pred = np.zeros_like(u_pred)
            # self.get_logger().info(f"dt_post_impact: {dt_post_impact}")
        elif self.impacted and dt_post_impact >= self.post_impact_backup_duration:
            self.get_logger().info('Impact passed, starting controller again!')
            self.impact_time = 0
            self.impacted = False
            self.has_impacted = True
            # recompute the straightline bezier
            xplus = np.mean(self.xplus_stack,axis=1).reshape(10,1)
            xmin = xplus.copy()
            xmin[0:3] += 100*xplus[3:6]
            # print(f"xplus: {xplus}")
            # print(f"xmin: {xmin}")
            self.plan['hvar'] = [np.linspace(t,t+100,5).reshape((1,5))]
            self.plan['rvar'] = [np.linspace(xplus.reshape((10,)),xmin.reshape((10,)),5).T]
            self.plan['drvar'] = [get_derivative_control_points_gurobi(self.plan['rvar'][0],1)]
            self.plan['dhvar'] = [get_derivative_control_points_gurobi(self.plan['hvar'][0],1)]
            self.plan['ids'] = ['none']
            self.plan['other_names'] = ['none']
            # print(f"rvar: {self.plan['rvar']}")
            # print(f"hvar: {self.plan['hvar']}")
            # print(f"drvar: {self.plan['drvar']}")
            # print(f"dhvar: {self.plan['dhvar']}")
            self.publish_plan()

        self.initial_guess = {'X': x_pred, 'U': u_pred}

        predicted_path_msg = Path()
        for idx in range(x_pred.shape[1]):
            # print(f"idx: {idx}, x_pred: {x_pred[:,idx]}")
            predicted_state = x_pred[:,idx]
            # Publish time history of the vehicle path
            predicted_pose_msg = vector2PoseMsg('map', predicted_state[0:3], np.array([1.0, 0.0, 0.0, 0.0]))
            predicted_path_msg.header = predicted_pose_msg.header
            predicted_path_msg.poses.append(predicted_pose_msg)
        self.predicted_path_pub.publish(predicted_path_msg)

        setpoint_path_msg = Path()
        for idx in range(len(setpoints)):
            setpoint = setpoints[idx]
            # print(f"setpoint[{idx}]: {setpoint[0:3]}")
            # Publish time history of the vehicle path
            setpoint_pose_msg = vector2PoseMsg('map', setpoint[0:3], np.array([1.0, 0.0, 0.0, 0.0]))
            setpoint_path_msg.header = setpoint_pose_msg.header
            setpoint_path_msg.poses.append(setpoint_pose_msg)
        self.reference_path_pub.publish(setpoint_path_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_rate_setpoint(u_pred)
        
        # print(f"Time elapsed: {time.time() - t0}")
        if time.time() - t0 > self.timer_period:
            self.get_logger().info(f"LOOP TOOK TOO LONG: {time.time() - t0} (timer period: {self.timer_period})")
            
    def add_set_plan_callback(self, request, response):
        self.get_logger().info('Received request')
        # self.plan = BezierPlan2NumpyArray(request.plan)
        self.plan = VerboseBezierPlan2NumpyArray(request.plan)
        # print some info
        self.get_logger().info(f"Number of bezier segments: {len(self.plan['rvar'])}")
        self.get_logger().info(f"Number of control points: {self.plan['rvar'][0].shape[1]}")
        self.get_logger().info(f"Segment ids: {self.plan['ids']}")
        # create the entire path and publish it to rviz
        self.publish_plan()

        return response
    
    def publish_plan(self):
        try:
            N = 100
            ts = np.linspace(self.plan['hvar'][0][0,0],self.plan['hvar'][-1][0,-1],N)
            entire_path_msg = Path()
            for t in ts:
                plani = interpolate_bezier(self.plan,t)
                posei = vector2PoseMsg('map', np.array([plani['q'][0], plani['q'][1], -0.01]), np.array([1.0, 0.0, 0.0, 0.0]))
                entire_path_msg.header = posei.header
                entire_path_msg.poses.append(posei)
            self.get_logger().info('Publishing the entire path')
            self.entire_path_pub.publish(entire_path_msg)
        except Exception as e:
            self.get_logger().info(f"Could not publish the entire path: {e}")

    def execute_plan_callback(self, msg):
        self.get_logger().info('Starting executing the plan')
        # check if self.plan is set
        if msg.data:
            if self.plan is None:
                self.get_logger().info('No plan set, cannot execute')
                self.started = False
            else:
                self.started = True
                self.start_time = Clock().now().nanoseconds / 1000
        


def main(args=None):
    rclpy.init(args=args)
    spacecraft_mpc = SpacecraftCleanMPC()
    rclpy.spin(spacecraft_mpc)

    spacecraft_mpc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
