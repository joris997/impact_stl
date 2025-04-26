#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import rclpy
import numpy as np
import time
import os

from planner.utilities.beziers import value_bezier, eval_t, get_derivative_control_points_gurobi
from impact_stl.planner.utilities.read_write_plan import csv_to_plan

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
# from impact_stl.controller.rate_mpc import SpacecraftRateMPC
from impact_stl.controllers.rate_mpc import SpacecraftRateMPC
from impact_stl.controllers.rate_impact_mpc import SpacecraftRateImpactMPC
from impact_stl.helpers.helpers import vector2PoseMsg, BezierCurve2NumpyArray, \
                            BezierPlan2NumpyArray, interpolate_bezier, VerboseBezierPlan2NumpyArray,\
                            Quaternion2Euler, Euler2Quaternion

class SpacecraftImpactMPC(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Creating SpacecraftImpactMPC node')
        # the object is an actual robot, so it has a namespace that we need 
        # for properly timing the replanning
        self.robot_name = self.get_namespace()
        self.object_ns = self.declare_parameter('object_ns', '/crackle').value
        self.scenario_name = self.declare_parameter('scenario_name', 'catch_throw').value
        self.enable_cbf = self.declare_parameter('enable_cbf', False).value
        self.get_logger().info(f"robot_name: {self.robot_name}, object_ns: {self.object_ns}, enable_cbf: {self.enable_cbf}")

        # get initial state from passed parameters
        self.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            1.0, 0.0, 0.0, 0.0]).reshape(10, 1)
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
        
        # Impact detector
        self.impact_detected_sub = self.create_subscription(
            StampedBool,
            'impact_stl/impact_detected',
            self.impact_detected_callback,
            RELIABLE_QOS)
        self.impacted = False
        self.impact_time = 0

        # Global reset detector
        self.global_reset_sub = self.create_subscription(
            StampedBool,
            '/global_reset',
            self.global_reset_callback,
            RELIABLE_QOS)

        # controller heuristics
        self.post_impact_backup_duration = 3.0  # how long we turn the contoller off after an impact
        self.t_object_coming = np.inf           # time when the object is coming to our impact point
        self.object_coming_wait = 3.0           # how long we wait with replanning after the object is coming

        # for long specs, object_coming_wait: 3, stacksize can be larger: more precise
        # for short specs, object_coming_wait: 1, stacksize has to be smaller
        
        # get the plan of the object from the csv file
        # this plan also never changes, so we can load it and use it forever :)
        package_share_directory = get_package_share_directory('impact_stl')
        plans_path = os.path.join(package_share_directory)
        try:
            self.get_logger().info(f"getting the plan for object {self.object_ns}")
            rvar,hvar,ids,other_names = csv_to_plan(robot_name=self.object_ns,
                                                    scenario_name=self.scenario_name,
                                                    path=plans_path)
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
        self.replanned = False  # Flag to start replanning when we arrive at pre-impact Bezier

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', NORMAL_QOS)
        self.publisher_rates_setpoint = self.create_publisher(VehicleRatesSetpoint, 'fmu/in/vehicle_rates_setpoint', NORMAL_QOS)
        self.predicted_path_pub = self.create_publisher(Path, 'impact_stl/predicted_path', 10)
        self.reference_path_pub = self.create_publisher(Path, "impact_stl/reference_path", 10)
        self.entire_path_pub = self.create_publisher(Path, "impact_stl/entire_path", 10)
        self.publisher_recompute_local_plan = self.create_publisher(StampedBool, 'impact_stl/recompute_local_plan', RELIABLE_QOS)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # Create Spacecraft and controller objects
        self.model = SpacecraftRateModel()
        self.mpc = SpacecraftRateMPC(self.model,Tf=1.0,N=10,add_cbf=self.enable_cbf)
        self.impact_mpc = SpacecraftRateImpactMPC(self.model,Tf=1.0,N=10)
        self.initial_guess = {'X': None, 'U': None}

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
        self.replanned = False
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

    def vehicle_status_callback(self, msg):
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def impact_detected_callback(self, msg):
        self.get_logger().info('Impact received')
        self.impacted = True
        self.impact_time = Clock().now().nanoseconds / 1000

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

    def get_pre_idx_and_tI(self,t):
        # if plan0 is pre, and plan1 is post, that means an impact has occurred
        # in the horizon, and we need to use an impact-mpc
        try:
            pre_indices = [i for i, x in enumerate(self.plan['ids']) if x == 'pre']
            pre_tIs = [self.plan['hvar'][i][0,-1] for i in pre_indices]
            # impacts may only occur in the future, so we find the first for which tI > t
            pre_idx = next((i for i, tI in enumerate(pre_tIs) if tI > t), len(pre_tIs)-1)
            tI = pre_tIs[pre_idx]
            # print(f"pre_indices: {pre_indices}, pre_tIs: {pre_tIs}")
            # print(f"pre_idx: {pre_idx}, tI: {tI}")
        except:
            pre_tIs = [0.0]
            pre_idx = -1
            tI = np.inf
        return pre_idx, tI, pre_tIs


    def get_setpoints(self):
        t = (Clock().now().nanoseconds / 1000 - self.start_time) / 1e6
        setpoints = []
        times = []
        impact_idx = None
        xpre = None
        xpost = None
        tI = None
        weights = {'Q': None, 'Q_e': None, 'R': None}

        # if we haven't started the simulation, we just keep position at the setpoint
        if not self.started:
            setpoint = self.x0
            # print(f"setpoint: {setpoint.T}")
            # self.get_logger().info(f"setpoint in get_setpoints: {setpoint}")
            for i in range(self.mpc.N+1):
                ti = t+i*self.mpc.dt
                setpoints.append(setpoint)
                times.append(ti)
            
            return setpoints, times, impact_idx, xpre, xpost, weights, tI
        # we started the simulation
        else:
            # self.get_logger().info(f"planner time: {t}")
            plan0 = interpolate_bezier(self.plan,t)
            tf = t+self.mpc.Tf
            planf = interpolate_bezier(self.plan,tf)

            # print(f"id: {plan0['id']}")
            self.idx, _ = eval_t(self.plan['hvar'], t)
            self.pre_idx, tI, pre_tIs = self.get_pre_idx_and_tI(t)
            # self.get_logger().info(f"pre_idx: {self.pre_idx}, tI: {tI}, pre_tIs: {pre_tIs}")
            t_near_a_tI = any([np.abs(pre_tI - t) < 5e-1 for pre_tI in pre_tIs])

            # 1. find the index of the pre-impact bezier of the object
            for idx_I in range(len(self.plan_object['ids'])):
                if tI > self.plan_object['hvar'][idx_I][0,-1]-1e-4 and tI < self.plan_object['hvar'][idx_I][0,-1]+1e-4:
                    if self.plan_object['ids'][idx_I] == 'pre' and self.plan_object['other_names'][idx_I] in self.robot_name:
                        # self.get_logger().info(f"pre-impact bezier found for the object at index {idx_I} (robot {self.robot_name})")
                        break
            # 2. find the index of the current bezier of the object
            for idx_Now in range(len(self.plan_object['ids'])):
                if t > self.plan_object['hvar'][idx_Now][0,0] and t < self.plan_object['hvar'][idx_Now][0,-1]:
                    # print(f"current bezier for the object at index {idx_Now}")
                    break
            # self.get_logger().info(f"plan_object['ids'][idx_Now]: {self.plan_object['ids'][idx_Now]}")
            # self.get_logger().info(f"plan_object['ids'][idx_I]: {self.plan_object['ids'][idx_I]}")
            # 2a. check if all 'ids' from idx_Now to idx_I are 'post' or 'none', if that's so, we could replan
            # self.get_logger().info(f"idx_Now: {idx_Now}, idx_I: {idx_I}, self.idx: {self.idx}")
            going_straight = True
            for idx in range(idx_Now,idx_I):
                # self.get_logger().info(f"plan_object['ids'][{idx}]: {self.plan_object['ids'][idx]}")
                if self.plan_object['ids'][idx] == 'pre':
                    going_straight = False
                    break
            # self.get_logger().info(f"going_straight: {going_straight}")

            # 3. if the current bezier of the object is the pre-impact bezier, we need to wait a bit before replanning
            #    to ensure the object is going straight for long enough
            if going_straight and plan0['id'] == 'pre' \
                    and self.t_object_coming == np.inf and not self.replanned:
                self.t_object_coming = t
                self.get_logger().info(f"that means we can replan, but we need to wait a bit")

            # 4. if the current bezier is a pre-impact bezier, we havent replanned yet, the object's 
            #    pre-impact bezier is the same as the current one, and the object is coming, we replan
            #    also do not replan 1 second before or one second after the desired impact time
            # TODO: this check should also work for short bezier segments of the object
            # TODO: check if idx_Now does not have any 'pre' segments afterwards until idx_I
            # self.get_logger().info(f"plan0['id']==pre: {plan0['id']=='pre'}, not replanned: {not self.replanned}, going_straight: {going_straight}, not t_near_a_tI: {not t_near_a_tI}, waited: {t - self.t_object_coming > self.object_coming_wait}")
            if plan0['id'] == 'pre' and not self.replanned and going_straight and \
                    t - self.t_object_coming > self.object_coming_wait and not t_near_a_tI:
                self.get_logger().info('Calling Replanning Service in ff_rate_mpc_impact')
                msg = StampedBool()
                msg.timestamp = int(Clock().now().nanoseconds / 1000)
                msg.t = t
                msg.data = True
                self.publisher_recompute_local_plan.publish(msg)
                self.replanned = True
                self.t_object_coming = np.inf

            # compute the nominal plan via interpolating the bezier curve
            # then, if an impact will occur, we just add that later at the right position
            for i in range(self.mpc.N+1):
                ti = t+i*self.mpc.dt
                plani = interpolate_bezier(self.plan,ti)

                setpoints.append(np.array([plani['q'][0], plani['q'][1], 0.0,
                                            plani['dq'][0], plani['dq'][1], 0.0,
                                            1.0, 0.0, 0.0, 0.0]).reshape(10,1))
                times.append(ti)

            # get the desired state just before and after the impact
            plan_pre = interpolate_bezier(self.plan,tI-(1e-6))
            plan_post = interpolate_bezier(self.plan,tI+(1e-6))
            setpoint_pre = np.array([plan_pre['q'][0], plan_pre['q'][1], 0.0,
                                     plan_pre['dq'][0], plan_pre['dq'][1], 0.0,
                                     1.0, 0.0, 0.0, 0.0]).reshape(10,1)
            setpoint_post = np.array([plan_post['q'][0], plan_post['q'][1], 0.0,
                                      plan_post['dq'][0], plan_post['dq'][1], 0.0,
                                      1.0, 0.0, 0.0, 0.0]).reshape(10,1)
            
            if t <= tI and tI <= tf-self.mpc.dt:
                self.get_logger().info("\nImpact in horizon!")
                # now insert setpoint_pre in the first index for which tI < t in times
                insert_idx = next((i for i, t in enumerate(times) if tI < t), len(times)-1)
                # print(f"insert_idx: {insert_idx}")
                # print(f"times before shenanigans: {times}")
                # remove the index after insert_idx
                del setpoints[insert_idx+1]
                del times[insert_idx+1]
                # and add the setpoint_pre and setpoint_post at insert_idx and insert_idx+1
                setpoints.insert(insert_idx,setpoint_pre)
                setpoints.insert(insert_idx+1,setpoint_post)
                times.insert(insert_idx,tI)
                times.insert(insert_idx+1,tI)
                # print(f"times after shenanigans: {times}")

                impact_idx = insert_idx
                xpre = setpoint_pre
                xpost = setpoint_post
                weights = {'Q': None, 'Q_e': None, 'R': None}
            
            # if tI has been in the horizon, but impacted=False,
            # we need to have the MPC focus on maintaining the last setpoint
            # and increase the weight on the velocity cost
            if tI <= t-2*self.mpc.dt and t - tI <= self.post_impact_backup_duration \
                and not self.impacted:
                print("Impact not yet occured, post-impact backup plan")
                # setpoints is plan_pre repeated N+1 times
                setpoints = [setpoint_pre for _ in range(self.mpc.N+1)]
                Q = np.diag([0e0, 0e0, 0e0, 8e0, 8e0, 8e0, 8e3])
                Q_e = np.diag([0e0, 0e0, 0e0, 8e0, 8e0, 8e0, 8e3])
                R = 2*np.diag([1e-3, 1e-3, 1e-3, 2e0, 2e0, 2e0])
                weights = {'Q': Q, 'Q_e': Q_e, 'R': R}
                # weights = {'Q': None, 'Q_e': None, 'R': None}
            
            return setpoints, times, impact_idx, xpre, xpost, weights, tI



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
        # print(f"x0: {x0}")

        # get the reference states and corresponding times in the horizon
        setpoints, times, impact_idx, xpre, xpost, weights, tI = self.get_setpoints()
        
        # solve the mpc
        if impact_idx is None:
            # now the initial guess has to be x_pred again, but check the size!
            if self.initial_guess['X'] is not None:
                self.initial_guess['X'] = self.initial_guess['X'][:,1::] if self.initial_guess['X'].shape[1] > self.mpc.N+1 else self.initial_guess['X']    
                self.initial_guess['U'] = self.initial_guess['U'][:,1::] if self.initial_guess['U'].shape[1] > self.mpc.N else self.initial_guess['U']
            

            t = 0 if times is None or not self.started else times[0]
            if self.plan is None:
                id = 'none'
            else:
                plan0 = interpolate_bezier(self.plan,t)
                id = plan0['id']
            # self.get_logger().info(f"t: {t}")
            self.get_logger().info(f"id: {id}") if self.enable_cbf else None
            if (id == 'pre' and tI - t <= 5) or not self.enable_cbf:
                enable_cbf = False
                xobj = None
            else:
                enable_cbf = True
                xobj = xobj
            self.get_logger().info(f"enable_cbf: {enable_cbf}") if self.enable_cbf else None
            x_pred, u_pred = self.mpc.solve(x0,setpoints,
                                            weights=weights,
                                            initial_guess=self.initial_guess,
                                            xobj=xobj,enable_cbf=enable_cbf,
                                            verbose=False)
        else:
            assert(xpre is not None)
            assert(xpost is not None)
            # now the initial guess can be the setpoints
            self.initial_guess['X'] = np.array([setpoints]).reshape(10, self.impact_mpc.N+1)
            # for the control input, if the previous mpc was a normal mpc, we need to add zeros 
            # where the impact will occur
            if self.initial_guess['U'].shape[1] < self.impact_mpc.N:
                self.initial_guess['U'] = np.insert(self.initial_guess['U'],impact_idx,np.zeros((6,)), axis=1)
            # print(f"setpoints:")
            # for i in range(len(setpoints)):
            #     print(f"[{i}]: {setpoints[i].T}")

            x_pred, u_pred = self.impact_mpc.solve(x0,xpre,xpost,
                                                   impact_idx,setpoints,times,
                                                   weights=weights,
                                                   initial_guess=self.initial_guess,
                                                   verbose=False)
            
        # if we just impacted the object, turn off the controller to prevent further pushing
        dt_post_impact = (Clock().now().nanoseconds / 1000 - self.impact_time)/1e6
        if self.impacted and dt_post_impact < self.post_impact_backup_duration:
            self.get_logger().info('Impact detected, stopping controller')
            # self.replanned = False
            u_pred = np.zeros_like(u_pred)
        if self.impacted and dt_post_impact >= self.post_impact_backup_duration:
            self.get_logger().info('Impact passed, starting controller again!')
            self.impact_time = 0
            self.impacted = False
            self.replanned = False

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
            self.get_logger().info(f"LOOP TOOK TOO LONG: {time.time() - t0} (timer_period: {self.timer_period})")
            
    def add_set_plan_callback(self, request, response):
        self.get_logger().info('Received request')
        # self.plan = BezierPlan2NumpyArray(request.plan)
        self.plan = VerboseBezierPlan2NumpyArray(request.plan)
        # print some info
        self.get_logger().info(f"Number of bezier segments: {len(self.plan['rvar'])}")
        self.get_logger().info(f"Number of control points: {self.plan['rvar'][0].shape[1]}")
        self.get_logger().info(f"Segment ids: {self.plan['ids']}")
        # create the entire path and publish it to rviz
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
        


def main(args=None):
    rclpy.init(args=args)
    spacecraft_mpc = SpacecraftImpactMPC()
    rclpy.spin(spacecraft_mpc)

    spacecraft_mpc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
