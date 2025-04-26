#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import rclpy
import numpy as np
import time

from impact_stl.helpers.beziers import value_bezier, eval_t

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
from my_msgs.msg import StampedBool

from impact_stl.models.spacecraft_rate_model import SpacecraftRateModel
# from impact_stl.controller.rate_mpc import SpacecraftRateMPC
from impact_stl.controllers.rate_mpc import SpacecraftRateMPC
from impact_stl.helpers.helpers import vector2PoseMsg, BezierCurve2NumpyArray, \
                            BezierPlan2NumpyArray, interpolate_bezier, VerboseBezierPlan2NumpyArray

class SpacecraftMPC(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Creating SpacecraftMPC node')
        # the object is an actual robot, so it has a namespace that we need 
        # for obstacle avoidance
        self.robot_name = self.get_namespace()
        self.object_ns = self.declare_parameter('object_ns', '/pop').value
        self.add_cbf = self.declare_parameter('add_cbf', False).value

        # get initial state from passed parameters
        self.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            1.0, 0.0, 0.0, 0.0]).reshape(10, 1)
        self.x0[0] = self.declare_parameter('x0', 0.0).value
        self.x0[1] = self.declare_parameter('y0', 0.0).value
        self.x0[2] = self.declare_parameter('z0', 0.0).value
        self.x0[3] = self.declare_parameter('vx0', 0.0).value
        self.x0[4] = self.declare_parameter('vy0', 0.0).value
        self.x0[5] = self.declare_parameter('vz0', 0.0).value

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
        
        # subscriber for the object for the CBF
        self.object_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{self.object_ns}/fmu/out/vehicle_local_position_gz',
            self.object_local_position_callback,
            NORMAL_QOS)
        
        # Bezier planner stuff
        self.execute_plan_sub = self.create_subscription(
            StampedBool,
            'impact_stl/execute_plan',
            self.execute_plan_callback,
            RELIABLE_QOS)
        
        self.control_mode_sub = self.create_subscription(
            StampedBool,
            'impact_stl/control_mode',
            self.control_mode_callback,
            RELIABLE_QOS)
        
        # Global reset listener
        self.global_reset_sub = self.create_subscription(
            StampedBool,
            '/global_reset',
            self.global_reset_callback,
            RELIABLE_QOS)
        
        # service for obtaining an initial and an updated motion plan!
        self.set_plan_srv = self.create_service(SetVerbosePlan, 'set_plan', self.add_set_plan_callback)
        self.start_time = 0     # Time when the plan starts, used to interpolate the Bezier curves to get the plan
        self.started = False    # Flag to start the plan, based on start_plan bool topic
        self.plan = None

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', NORMAL_QOS)
        self.publisher_rates_setpoint = self.create_publisher(VehicleRatesSetpoint, 'fmu/in/vehicle_rates_setpoint', NORMAL_QOS)
        self.predicted_path_pub = self.create_publisher(Path, 'impact_stl/predicted_path', 10)
        self.reference_path_pub = self.create_publisher(Path, "impact_stl/reference_path", 10)
        self.publisher_recompute_local_plan = self.create_publisher(StampedBool, 'impact_stl/recompute_local_plan', RELIABLE_QOS)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # Create Spacecraft and controller objects
        self.model = SpacecraftRateModel()
        self.mpc = SpacecraftRateMPC(self.model,Tf=1.0,N=10,add_cbf=self.add_cbf)

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])

        self.object_local_position = np.array([0.0, 0.0, 0.0])
        self.object_local_velocity = np.array([0.0, 0.0, 0.0])

    def global_reset_callback(self, msg):
        self.get_logger().info('Global reset received')
        self.started = False

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

    def object_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.object_local_position[0] = msg.x
        self.object_local_position[1] = -msg.y
        self.object_local_position[2] = -msg.z
        self.object_local_velocity[0] = msg.vx
        self.object_local_velocity[1] = -msg.vy
        self.object_local_velocity[2] = -msg.vz

    def vehicle_angular_velocity_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_angular_velocity[0] = msg.xyz[0]
        self.vehicle_angular_velocity[1] = -msg.xyz[1]
        self.vehicle_angular_velocity[2] = -msg.xyz[2]

    def vehicle_angular_velocity_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_angular_velocity[0] = msg.xyz[0]
        self.vehicle_angular_velocity[1] = -msg.xyz[1]
        self.vehicle_angular_velocity[2] = -msg.xyz[2]

    def vehicle_status_callback(self, msg):
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

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

        if not self.started:
            setpoint = self.x0
            for i in range(self.mpc.N+1):
                ti = t+i*self.mpc.dt
                setpoints.append(setpoint)
        else:
            for i in range(self.mpc.N+1):
                ti = t+i*self.mpc.dt
                plan = interpolate_bezier(self.plan, ti)
                setpoints.append(np.array([plan['q'][0], plan['q'][1], 0.0,
                                             plan['dq'][0], plan['dq'][1], 0.0,
                                             1.0, 0.0, 0.0, 0.0]).reshape(10,1))
        return setpoints

    def cmdloop_callback(self):
        t0 = time.time()
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
        xobj = np.array([self.object_local_position[0], self.object_local_position[1], self.object_local_position[2],
                         self.object_local_velocity[0], self.object_local_velocity[1], self.object_local_velocity[2],
                         1,0,0,0]).reshape(10, 1)
        
        print(f"Vehicle position: {x0[0:3].T}")
        # create an array of the reference states
        setpoints = self.get_setpoints()
        # print(f"Setpoint: {setpoints[0][0:3].T}")
        # solve the MPC, also pass the object state for the CBF
        x_pred, u_pred = self.mpc.solve(x0,setpoints,
                                        xobj=xobj,
                                        verbose=True)

        # ship it
        predicted_path_msg = Path()
        for idx in range(x_pred.shape[1]):
            predicted_state = x_pred[:,idx]
            # Publish time history of the vehicle path
            predicted_pose_msg = vector2PoseMsg('map', predicted_state[0:3], np.array([1.0, 0.0, 0.0, 0.0]))
            predicted_path_msg.header = predicted_pose_msg.header
            predicted_path_msg.poses.append(predicted_pose_msg)
        self.predicted_path_pub.publish(predicted_path_msg)

        setpoint_path_msg = Path()
        for idx in range(len(setpoints)):
            setpoint = setpoints[idx]
            # Publish time history of the vehicle path
            setpoint_pose_msg = vector2PoseMsg('map', setpoint[0:3], np.array([1.0, 0.0, 0.0, 0.0]))
            setpoint_path_msg.header = setpoint_pose_msg.header
            setpoint_path_msg.poses.append(setpoint_pose_msg)
        self.reference_path_pub.publish(setpoint_path_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_rate_setpoint(u_pred)
        
        # print(f"Time elapsed: {time.time() - t0}")
    
    def add_set_plan_callback(self, request, response):
        self.get_logger().info('Received request')
        # self.plan = BezierPlan2NumpyArray(request.plan)
        self.plan = VerboseBezierPlan2NumpyArray(request.plan)
        # print some info
        self.get_logger().info(f"Number of bezier segments: {len(self.plan['rvar'])}")
        self.get_logger().info(f"Number of control points: {self.plan['rvar'][0].shape[1]}")
        self.get_logger().info(f"Segment ids: {self.plan['ids']}")
        return response

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
        
    def control_mode_callback(self, msg):
        self.get_logger().info('Changing control mode')
        if msg.data:
            self.started = True
        if not msg.data:
            self.started = False


def main(args=None):
    rclpy.init(args=args)
    spacecraft_mpc = SpacecraftMPC()
    rclpy.spin(spacecraft_mpc)

    spacecraft_mpc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
