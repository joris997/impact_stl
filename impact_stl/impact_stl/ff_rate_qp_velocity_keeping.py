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
from impact_stl.controllers.rate_clf_qp import SpacecraftRateQP
from impact_stl.helpers.helpers import vector2PoseMsg, BezierCurve2NumpyArray, \
                            BezierPlan2NumpyArray, interpolate_bezier, VerboseBezierPlan2NumpyArray

class SpacecraftVelocityKeepingMPC(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Creating SpacecraftVelocityKeepingMPC node')

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
            'fmu/out/vehicle_attitude_gz',
            self.vehicle_attitude_callback,
            NORMAL_QOS)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position_gz',
            self.vehicle_local_position_callback,
            NORMAL_QOS)
        
        # Impact detector
        self.impact_detected_sub = self.create_subscription(
            StampedBool,
            'impact_stl/impact_detected',
            self.impact_detected_callback,
            RELIABLE_QOS)
        self.impacted = False
        self.impact_time = 0
        self.impact_cnt = 0
        
        # service for obtaining an initial and an updated motion plan!
        self.start_time = 0     # Time when the plan starts, used to interpolate the Bezier curves to get the plan
        self.started = False    # Flag to start the plan, based on start_plan bool topic
        self.plan = None

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', NORMAL_QOS)
        self.publisher_rates_setpoint = self.create_publisher(VehicleRatesSetpoint, 'fmu/in/vehicle_rates_setpoint', NORMAL_QOS)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # Keep track of a stack of current states
        stack_size = 100
        self.x0_stack = np.zeros((10, stack_size))

        # Create Spacecraft and controller objects
        self.model = SpacecraftRateModel()
        self.qp = SpacecraftRateQP(self.model,
                                    add_pos_clf=True,add_vel_clf=True,
                                    add_cbf=False)

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])

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

    def vehicle_status_callback(self, msg):
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def publish_rate_setpoint(self, u_pred):
        thrust_rates = u_pred
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

    def impact_detected_callback(self, msg):
        self.get_logger().info('Impact received')
        self.impacted = True
        self.impact_cnt += 1
        self.impact_time = Clock().now().nanoseconds / 1000

    def get_setpoints(self):
        return self.x0

    def cmdloop_callback(self):
        t0 = time.time()
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.direct_actuator = False
        offboard_msg.body_rate = True   # rate control
        self.publisher_offboard_mode.publish(offboard_msg)

        x0 = np.array([self.vehicle_local_position[0], self.vehicle_local_position[1], self.vehicle_local_position[2],
                       self.vehicle_local_velocity[0], self.vehicle_local_velocity[1], self.vehicle_local_velocity[2],
                       self.vehicle_attitude[0], self.vehicle_attitude[1], self.vehicle_attitude[2], self.vehicle_attitude[3]]).reshape(10, 1)
        
        # deal with a potential impact
        if self.impacted and (Clock().now().nanoseconds / 1000 - self.impact_time)/1e6 < 1:
            u_pred = np.zeros((6, 1))
        elif self.impacted:
            self.impacted = False
            self.x0 = x0
            u_pred = np.zeros((6, 1))
        else:
            # create an array of the reference states
            setpoint = self.get_setpoints()
            # if we haven't had an impact yet but we are armed we just
            # do velocity keeping to ensure that the object is at correct pos.
            # we set slack parameters for the CBF and CLFs. 0: on, bigM: off
            onoffs = {'onoff_H': 1000, 'onoff_V_pos': 1000, 'onoff_V_vel': 0}
            u_pred = self.qp.solve(x0,setpoint,
                                   onoff=onoffs,
                                   p_ref=setpoint[0:3],v_ref=setpoint[3:6],
                                   verbose=True)

        # ship it
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_rate_setpoint(u_pred)

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
    spacecraft_mpc = SpacecraftVelocityKeepingMPC()
    rclpy.spin(spacecraft_mpc)

    spacecraft_mpc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
