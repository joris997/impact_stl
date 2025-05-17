#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import numpy as np
import time
import rclpy
import casadi as cs
from rclpy.node import Node
from rclpy.clock import Clock
from impact_stl.helpers.qos_profiles import NORMAL_QOS, RELIABLE_QOS
import os
import cvxpy as cp

from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition

from my_msgs.msg import StampedBool

from ament_index_python.packages import get_package_share_directory

from impact_stl.planners.main_planner import MinimalClientAsync
from impact_stl.helpers.beziers import get_derivative_control_points_gurobi, get_derivative_control_points_cvxpy
from impact_stl.helpers.read_write_plan import csv_to_plan, plan_to_csv
from impact_stl.helpers.solve_two_body_impact import solve_two_body_impact
from impact_stl.helpers.plot_rvars_hvars import plot_rvars_hvars

class RePlanner(Node):
    def __init__(self):
        super().__init__('replanner')
        self.get_logger().info('Creating Replanner Node')
        
        self.minimal_client = MinimalClientAsync()

        # the object is an actual robot, so it has a namespace
        self.robot_name = self.get_namespace() if self.get_namespace() != '/' else 'pop'
        self.scenario_name = self.declare_parameter('scenario_name', 'throw_and_catch_exp').value
        self.object_ns = self.declare_parameter('object_ns', '/pop').value
        self.gz = self.declare_parameter('gz', True).value
        print(f"object_ns: {self.object_ns}")

        gz_suffix = '_gz' if self.gz else ''
        # object subscribers
        self.object_attitude_sub = self.create_subscription(
            VehicleAttitude,
            f'{self.object_ns}/fmu/out/vehicle_attitude{gz_suffix}',
            self.object_attitude_callback,
            NORMAL_QOS)
        self.object_angular_vel_sub = self.create_subscription(
            VehicleAngularVelocity,
            f'{self.object_ns}/fmu/out/vehicle_angular_velocity{gz_suffix}',
            self.object_angular_velocity_callback,
            NORMAL_QOS)
        self.object_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{self.object_ns}/fmu/out/vehicle_local_position{gz_suffix}',
            self.object_local_position_callback,
            NORMAL_QOS)
        
        # robot subscribers
        self.robot_attitude_sub = self.create_subscription(
            VehicleAttitude,
            'fmu/out/vehicle_attitude',
            self.robot_attitude_callback,
            NORMAL_QOS)
        self.robot_angular_vel_sub = self.create_subscription(
            VehicleAngularVelocity,
            'fmu/out/vehicle_angular_velocity',
            self.robot_angular_velocity_callback,
            NORMAL_QOS)
        self.robot_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.robot_local_position_callback,
            NORMAL_QOS)
        
        # replan subscriber
        self.replan_sub = self.create_subscription(
            StampedBool,
            'impact_stl/recompute_local_plan',
            self.recompute_local_plan_callback,
            RELIABLE_QOS)
        
        # get the original plan from the csv file
        package_share_directory = get_package_share_directory('impact_stl')
        plans_path = os.path.join(package_share_directory)
        self.rvars,self.hvars,self.idvars,self.other_names = csv_to_plan(self.robot_name,
                                                                         scenario_name=self.scenario_name,
                                                                         path=plans_path)
        # get the plan of the object from the csv file
        try:
            self.orvars,self.ohvars,self.oidvars,self.oother_names = csv_to_plan(self.object_ns,
                                                                                 scenario_name=self.scenario_name,
                                                                                 path=plans_path)
        except Exception as e:
            print(f"Could not find plan for object {self.object_ns}")
            print(f"Error: {e}")

        self.drvars = [get_derivative_control_points_gurobi(rvar) for rvar in self.rvars]
        self.dhvars = [get_derivative_control_points_gurobi(hvar) for hvar in self.hvars]
        
        # From the .sdf file:
        # /home/px4space/PX4/PX4-Space-Systems/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/2d_spacecraft/2d_spacecraft.sdf
        self.robot_radius = 0.20
        self.robot_mass = 16.8
        self.object_radius = 0.20
        self.object_mass = 16.8
        self.m1 = (self.robot_mass-self.object_mass)/(self.robot_mass+self.object_mass)
        self.m2 = (2*self.object_mass)/(self.robot_mass+self.object_mass)
        self.m3 = (2*self.robot_mass)/(self.robot_mass+self.object_mass)
        self.m4 = (self.object_mass-self.robot_mass)/(self.robot_mass+self.object_mass)

        # position and velocity variables that are updated with the subscriber calls
        self.object_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.object_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.object_local_position = np.array([0.0, 0.0, 0.0])
        self.object_local_velocity = np.array([0.0, 0.0, 0.0])
        # keep track of a stack of velocities to interpolate
        #TODO: if this is linked to an EKF we should use that, but the EKF needs to consider impacts
        stack_size = 10 # was 30
        self.planner_time = 0.
        self.object_local_velocity_stack = np.zeros((3,stack_size))
        self.object_local_position_stack = np.zeros((3,stack_size))
        self.object_local_position_time_stack = np.zeros((1,stack_size))

        self.robot_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.robot_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_local_position = np.array([0.0, 0.0, 0.0])
        self.robot_local_velocity = np.array([0.0, 0.0, 0.0])

        self.Ncalls = 1
        self.Ncallbacks = 0
        self.verbose = True

        self.get_logger().info('Finished Replanner Node')

        
    def object_attitude_callback(self, msg):
        self.object_attitude[0] = msg.q[0]
        self.object_attitude[1] = msg.q[1]
        self.object_attitude[2] = -msg.q[2]
        self.object_attitude[3] = -msg.q[3]
    def robot_attitude_callback(self, msg):
        self.robot_attitude[0] = msg.q[0]
        self.robot_attitude[1] = msg.q[1]
        self.robot_attitude[2] = -msg.q[2]
        self.robot_attitude[3] = -msg.q[3]

    def object_angular_velocity_callback(self, msg):
        self.object_angular_velocity[0] = msg.xyz[0]
        self.object_angular_velocity[1] = -msg.xyz[1]
        self.object_angular_velocity[2] = -msg.xyz[2]
    def robot_angular_velocity_callback(self, msg):
        self.robot_angular_velocity[0] = msg.xyz[0]
        self.robot_angular_velocity[1] = -msg.xyz[1]
        self.robot_angular_velocity[2] = -msg.xyz[2]

    def object_local_position_callback(self, msg):
        self.object_local_position_time_stack[:,0:-1] = self.object_local_position_time_stack[:,1:]
        self.object_local_position_time_stack[:,-1] = msg.timestamp
        self.object_local_position[0] = msg.x
        self.object_local_position[1] = -msg.y
        self.object_local_position[2] = -msg.z
        self.object_local_position_stack[:,0:-1] = self.object_local_position_stack[:,1:]
        self.object_local_position_stack[:,-1] = self.object_local_position
        self.object_local_velocity[0] = msg.vx
        self.object_local_velocity[1] = -msg.vy
        self.object_local_velocity[2] = -msg.vz
        self.object_local_velocity_stack[:,0:-1] = self.object_local_velocity_stack[:,1:]
        self.object_local_velocity_stack[:,-1] = self.object_local_velocity

        # self.Ncallbacks += 1
        # if self.Ncallbacks % 100 == 0:
        #     self.get_logger().info(f"AAAAAAAAAAAA{self.Ncallbacks}")

    def robot_local_position_callback(self, msg):
        self.robot_local_position[0] = msg.x
        self.robot_local_position[1] = -msg.y
        self.robot_local_position[2] = -msg.z
        self.robot_local_velocity[0] = msg.vx
        self.robot_local_velocity[1] = -msg.vy
        self.robot_local_velocity[2] = -msg.vz
        
    
    def recompute_local_plan_callback(self, msg):
        # based on the position and velocities of the robot and obstacle
        # plus the desired post-impact positions and velocities of the obstacle
        # recompute the pre- and post-impact Beziers to deal with the sizes
        self.get_logger().info('Recomputing local plan for pre- and post- impact Bezier')
        self.planner_time = msg.t
        self.get_logger().info(f"planner_time: {self.planner_time}")
        self.solve_replan_cvxpy()
        self.get_logger().info("Local plan recomputed")

        if msg.data:
            self.get_logger().info('Sending plan')
            self.minimal_client.send_request(self.re_rvars, self.re_hvars, self.re_idvars, self.re_other_names)
            self.get_logger().info('Plan received')

    def compute_object_pre_post_state(self,pre_idx,tI):
        # obtain the pre-impact state of the object (which will be in the future)
        # and the desired post-impact state of the object, given the specification on the object

        # first, we obtain the velocity which is just the smoothed current velocity
        #TODO: there seems to be an acceleration remaining in the robot after impacts
        #TODO: to that end, we're going to compute it from position, which is always correct (at least in rviz and gz)
        dts = np.array([(self.object_local_position_time_stack[0,i+1]-self.object_local_position_time_stack[0,i])/1e6 \
                        for i in range(self.object_local_position_time_stack.shape[1]-1)])
        self.get_logger().info(f"dts: {dts}") if self.verbose else None
        dxs_from_pos = np.zeros((self.object_local_position_stack.shape[0],dts.shape[0]))
        for i in range(dts.shape[0]):
            dxs_from_pos[:,i] = (self.object_local_position_stack[:,i+1]-self.object_local_position_stack[:,i])/dts[i]
        self.get_logger().info(f"dx stack[0]:          {self.object_local_velocity_stack[0,:]}") if self.verbose else None
        self.get_logger().info(f"dx from pos stack[0]: {dxs_from_pos[0,:]}") if self.verbose else None

        dxObject0 = np.mean(self.object_local_velocity_stack,axis=1)
        self.get_logger().info(f"dxObject0 from vel: {dxObject0}") if self.verbose else None
        dxObject0 = np.mean(dxs_from_pos,axis=1)
        self.get_logger().info(f"dxObject0 from pos: {dxObject0}") if self.verbose else None
        dxObjectI = dxObject0[0:2] # constant velocity assumption
        #! change
        dxObjectI = np.array([dxObjectI[0],dxObjectI[1]])   # SITL
        # dxObjectI = np.array([-dxObjectI[1],dxObjectI[0]])  # HW
        
        # then we obtain the position which is the current position plus the velocity times the time
        xObject0 = self.object_local_position[0:2]
        #! change
        xObject0 = np.array([xObject0[0],xObject0[1]])    # SITL
        # xObject0 = np.array([-xObject0[1],xObject0[0]]) # HW
        self.get_logger().info(f"xObject0: {xObject0}") if self.verbose else None
        xObjectI = xObject0 + (tI - self.planner_time) * dxObjectI

        # then we obtain the desired post-impact state of the object
        self.get_logger().info(f"") if self.verbose else None
        pre_idxs_obj = [i for i, x in enumerate(self.oidvars) if x == 'pre']
        self.get_logger().info(f"pre_idxs_obj: {pre_idxs_obj}") if self.verbose else None
        other_name_idx_obj = [self.oother_names[idx] for idx in pre_idxs_obj]
        self.get_logger().info(f"other_name_idx_obj: {other_name_idx_obj}") if self.verbose else None
        for idx in range(len(other_name_idx_obj)):
            # is the pre-impact bezier of the object associated to this robot?
            if other_name_idx_obj[idx] in self.robot_name and \
                    self.ohvars[pre_idxs_obj[idx]][0,-1] >= tI-1e-4 and self.ohvars[pre_idxs_obj[idx]][0,-1] <= tI+1e-4:
                self.get_logger().info(f"Found the pre-impact bezier of the object") if self.verbose else None
                self.get_logger().info(f"idx: {other_name_idx_obj[idx]}") if self.verbose else None
                pre_idx_obj = pre_idxs_obj[idx]
                self.get_logger().info(f"pre_idx_obj: {pre_idx_obj}") if self.verbose else None
                break
        # then obtain the desired state of the object after free-floating
        # which is either the next pre-impact state or the last state
        if max(pre_idxs_obj) > pre_idx_obj:
            post_idx_obj = [idx for idx in pre_idxs_obj if idx > pre_idx_obj][0]
        else:
            post_idx_obj = len(self.orvars)-1
        self.get_logger().info(f"post_idx_obj: {post_idx_obj}") if self.verbose else None
        xObjectI_next = self.orvars[post_idx_obj][0:2,-1]
        #! don't change between sitl and hw
        xObjectI_next = xObjectI_next # SITL
        # xObjectI_next = np.array([-xObjectI_next[1],xObjectI_next[0]]) # HW
        self.get_logger().info(f"xObjectI_next: {xObjectI_next}") if self.verbose else None
        t_next = self.ohvars[post_idx_obj][0,-1]
        self.get_logger().info(f"t_next: {t_next}") if self.verbose else None
        self.get_logger().info(f"(t_next - tI): {t_next - tI}") if self.verbose else None
        self.get_logger().info(f"(xObjectI_next - xObjectI): {xObjectI_next - xObjectI}")
        dxObjectI_post = (xObjectI_next - xObjectI) / (t_next - tI)

        self.get_logger().info(f"\n") if self.verbose else None
        self.get_logger().info(f"xObjectI: {xObjectI}") if self.verbose else None
        self.get_logger().info(f"dxObjectI: {dxObjectI}") if self.verbose else None
        self.get_logger().info(f"dxObjectI_post: {dxObjectI_post}") if self.verbose else None

        return xObjectI, dxObjectI, dxObjectI_post


    def solve_replan(self):
        t_start = time.time()
        # create planning problem
        self.get_logger().info("Creating casadi replanner problem")
        ocp = cs.Opti()
        n_cp = 5
        rvars = [ocp.variable(2,n_cp) for _ in range(2)] # was 1,4
        hvars = [ocp.variable(1,n_cp) for _ in range(2)]

        # find idx where idvars is 'pre'
        pre_idxs = [i for i, x in enumerate(self.idvars) if x == 'pre']
        self.get_logger().info(f"pre_idxs: {pre_idxs}") if self.verbose else None
        pre_tIs = [self.hvars[idx][0,-1] for idx in pre_idxs]
        self.get_logger().info(f"pre_tIs: {pre_tIs}") if self.verbose else None
        # impacts may only occur in the future, so we find the first for which tI > t
        pre_idx = next((pre_idxs[i] for i, tI in enumerate(pre_tIs) if tI > self.planner_time), len(pre_tIs)-1)
        self.get_logger().info(f"pre_idx: {pre_idx}") if self.verbose else None

        t0 = self.hvars[pre_idx][0,0]
        # t0 = (Clock().now().nanoseconds / 1000 - self.start_time) / 1e6
        tI = self.hvars[pre_idx][0,-1]
        tf = self.hvars[pre_idx+1][0,-1]

        # TODO: make this able to deal with the noisy velocity measurements
        xObjectI, dxObjectI, dxObjectI_post = self.compute_object_pre_post_state(pre_idx,tI)
        dxI_init = np.array([self.drvars[pre_idx][0,-1]/self.dhvars[pre_idx][0,-1],
                             self.drvars[pre_idx][1,-1]/self.dhvars[pre_idx][0,-1]])
        # the initial angle is using the velocities but we need to add pi
        # because we impact on the other side of the circle as where the vector points 
        theta_init = np.arctan2(dxI_init[1],dxI_init[0]) + np.pi
        self.get_logger().info(f"init guesses, dxI_init: {dxI_init}, theta_init: {theta_init}") if self.verbose else None
        # then solve the two-body impact problem to obtain 
        # xI, dxI, xI_post, dxI_post = solve_two_body_impact(xObjectI,dxObjectI,dxObjectI_post,
        #                                                    dx_init_guess=dxI_init,
        #                                                    theta_init_guess=theta_init,
        #                                                    logger=self.get_logger())
        # or we can try to use the simple solution with theta_init and dxI_init
        # xI = xObjectI + 0.4*np.array([np.cos(theta_init),np.sin(theta_init)])
        dxI = -(1/0.99)*(dxObjectI - dxObjectI_post)
        theta_close = np.arctan2(dxI[1],dxI[0]) + np.pi
        xI = xObjectI + 0.40*np.array([np.cos(theta_close),np.sin(theta_close)])
        xI_post = xI
        dxI_post = dxObjectI
        self.get_logger().info(f"xI: {xI}, dxI: {dxI}, xI_post: {xI_post}, dxI_post: {dxI_post}") if self.verbose else None
        self.get_logger().info(f"dt after 2bi {time.time()-t_start}") if self.verbose else None

        # Bezier derivative properties
        drvars, ddrvars = [], []
        dhvars, ddhvars = [], []
        for idx in range(len(rvars)):
            drvars.append(get_derivative_control_points_gurobi(rvars[idx]))
            ddrvars.append(get_derivative_control_points_gurobi(rvars[idx],der_order=2))
            dhvars.append(get_derivative_control_points_gurobi(hvars[idx]))
            ddhvars.append(get_derivative_control_points_gurobi(hvars[idx],der_order=2))
        # increasing time
        for idx in range(len(dhvars)): 
            for i in range(dhvars[idx].shape[1]):
                ocp.subject_to(dhvars[idx][0,i] >= 1e-1)
        
        # continuity
        ocp.subject_to(rvars[0][:,-1] == rvars[-1][:,0])
        ocp.subject_to(hvars[0][:,-1] == hvars[-1][:,0])

        # velocity constraints
        # for idx in range(len(drvars)):
        #     for d in range(2):
        #         for i in range(drvars[idx].shape[1]):
        #             ocp.subject_to(drvars[idx][d,i] <= 0.35*dhvars[idx][0,i])
        #             ocp.subject_to(drvars[idx][d,i] >= -0.35*dhvars[idx][0,i])

        # # stay within workspace bounds
        # #! enable this if on hardware
        for idx in range(len(rvars)):
            for cp in range(rvars[idx].shape[1]):
                ocp.subject_to(rvars[idx][0,cp] >= 0 + 0.3)
                ocp.subject_to(rvars[idx][0,cp] <= 4 - 0.3)
                ocp.subject_to(rvars[idx][1,cp] >= -1.75 + 0.3)
                ocp.subject_to(rvars[idx][1,cp] <=  1.75 - 0.3)

        # initial state, now we use the most recent robot state
        x0 = self.robot_local_position[0:2]
        self.get_logger().info(f"current state, x0: {x0}") if self.verbose else None
        dx0 = self.robot_local_velocity[0:2]
        self.get_logger().info(f"current state, dx0: {dx0}") if self.verbose else None
        ocp.subject_to(rvars[0][:,0] == x0)
        ocp.subject_to(hvars[0][:,0] == t0)
        ocp.subject_to(drvars[0][:,0] == dx0*dhvars[0][:,0])
        # final state
        ocp.subject_to(rvars[-1][:,-1] == self.rvars[pre_idx+1][0:2,-1])
        ocp.subject_to(hvars[-1][:,-1] == self.hvars[pre_idx+1][:,-1])
        ocp.subject_to(drvars[-1][:,-1] == self.drvars[pre_idx+1][0:2,-1])
        ocp.subject_to(dhvars[-1][:,-1] == self.dhvars[pre_idx+1][:,-1])

        # impact condition, desired post-impact velocity of the object
        # this is equivalent to the impact constraints, since we've already
        # solved the two-body impact problem
        ocp.subject_to(hvars[0][:,-1] == tI)
        hard = True
        if hard:
            # ocp.subject_to(rvars[0][:,-1] == xI)
            # ocp.subject_to(rvars[-1][:,0] == xI_post)
            # ocp.subject_to(drvars[0][:,-1] == dxI*dhvars[0][:,-1])
            # ocp.subject_to(drvars[-1][:,0] == dxI_post*dhvars[-1][:,0])

            # relaxed but linear
            delta = ocp.variable()
            ocp.subject_to(delta >= 0)
            ocp.subject_to(rvars[0][:,-1] <= xI+delta)
            ocp.subject_to(rvars[0][:,-1] >= xI-delta)
            ocp.subject_to(rvars[-1][:,0] <= xI_post+delta)
            ocp.subject_to(rvars[-1][:,0] >= xI_post-delta)
            ocp.subject_to(drvars[0][:,-1] <= dxI*dhvars[0][:,-1]+delta)
            ocp.subject_to(drvars[0][:,-1] >= dxI*dhvars[0][:,-1]-delta)
            ocp.subject_to(drvars[-1][:,0] <= dxI_post*dhvars[-1][:,0]+delta)
            ocp.subject_to(drvars[-1][:,0] >= dxI_post*dhvars[-1][:,0]-delta)
        else:
            # soft version
            delta_r_pre, delta_r_post, delta_dr_pre, delta_dr_post = ocp.variable(), ocp.variable(), ocp.variable(), ocp.variable()
            ocp.subject_to(cs.sumsqr(rvars[0][:,-1] - xI) <= delta_r_pre)
            ocp.subject_to(cs.sumsqr(rvars[-1][:,0] - xI_post) <= delta_r_post)
            ocp.subject_to(cs.sumsqr(drvars[0][:,-1] - dxI*dhvars[0][:,-1]) <= delta_dr_pre)
            ocp.subject_to(cs.sumsqr(drvars[-1][:,0] - dxI_post*dhvars[-1][:,0]) <= delta_dr_post)

        # cost
        cost = 0
        for idx in range(len(ddhvars)):
            for i in range(ddhvars[idx].shape[1]):
                for d in range(2):
                    cost += ddrvars[idx][d,i]*ddrvars[idx][d,i]
                cost += ddhvars[idx][0,i]*ddhvars[idx][0,i]

        if not hard:
            cost += 1e4*(delta_r_pre + delta_r_post + delta_dr_pre + delta_dr_post)
        else:
            cost += 1e4*delta
        ocp.minimize(cost)
        # self.get_logger().info("added casadi cost")

        # set solver method
        # opts = {'ipopt.print_level': 0,
        #         'ipopt.tol': 1e-2,
        #         'ipopt.max_iter': 100,
        #         'print_time': 0, 'ipopt.sb': 'no'}
        # ocp.solver('ipopt',opts)
        
        # ocp.solver('qpoases')

        qp_opts = {
            # 'max_iter': 5,
            'error_on_fail': False,
            'printLevel': "none",
            # 'print_header': False,
            # 'print_iter': False
        }
        sqp_opts = {
            'max_iter': 10,
            'qpsol': 'qpoases',
            'convexify_margin': 1e-4,
            'print_header': False,
            'print_time': False,
            'print_iteration': False,
            'qpsol_options': qp_opts
        }
        ocp.solver('sqpmethod', sqp_opts)

        self.get_logger().info(f"dt after casadi setup {time.time()-t_start}")

        # solve
        t0 = time.time()
        sol = ocp.solve()
        self.get_logger().info(f"dt after solving {time.time()-t_start}") if self.verbose else None

        rvars = [sol.value(rvar).reshape((2,n_cp)) for rvar in rvars]
        hvars = [sol.value(hvar).reshape((1,n_cp)) for hvar in hvars]
        # pad rvars with an extra row of zeros (because there is orientation)
        rvars = [np.vstack([rvar,np.zeros((1,rvar.shape[1]))]) for rvar in rvars]

        # construct replanned rvars, hvars, idvars, other_names
        # we replace self.rvars[pre_idx] with rvars[0] and self.rvars[pre_idx+1] with rvars[1]
        self.re_rvars = self.rvars.copy()
        self.re_hvars = self.hvars.copy()
        self.re_idvars = self.idvars.copy()
        self.re_other_names = self.other_names.copy()
        self.re_rvars[pre_idx] = rvars[0]
        self.re_rvars[pre_idx+1] = rvars[1]
        self.re_hvars[pre_idx] = hvars[0]
        self.re_hvars[pre_idx+1] = hvars[1]

        # plot it (this takes a bunch of time! definately remove this before experiments)
        name = self.robot_name[1:]
        # plot_rvars_hvars(self.rvars,self.hvars,"/home/px4space/space_ws/replans",f"{name}_pre.png")
        # plot_rvars_hvars(self.re_rvars,self.re_hvars,"/home/px4space/space_ws/replans",f"{name}_{self.Ncalls}_post.png")

        # write it (this takes little time and is required for plotting afterwards)
        write_rvars = [rvars[0],rvars[1]]
        write_hvars = [hvars[0],hvars[1]]
        write_ids = ['pre','post']
        write_other_names = [self.object_ns[1:],self.object_ns[1:]]
        home_dir = os.path.expanduser("~")
        replans_path = os.path.join(home_dir, "space_ws", "replans")
        plan_to_csv(write_rvars, write_hvars, write_ids, write_other_names,
                    robot_name=f"{self.robot_name}_{self.Ncalls}",
                    scenario_name=self.scenario_name,
                    path=replans_path)

        self.Ncalls += 1
        self.get_logger().info(f"The whole ordeal took {time.time()-t_start} seconds")


    def solve_replan_cvxpy(self):
        t_start = time.time()
        self.get_logger().info("Creating CVXPY replanner problem")
        
        n_cp = 5
        rvars = [cp.Variable((2, n_cp)) for _ in range(2)]
        hvars = [cp.Variable((1, n_cp)) for _ in range(2)]
        
        # Find index of pre-impact segment
        pre_idxs = [i for i, x in enumerate(self.idvars) if x == 'pre']
        pre_tIs = [self.hvars[idx][0, -1] for idx in pre_idxs]
        pre_idx = next((pre_idxs[i] for i, tI in enumerate(pre_tIs) if tI > self.planner_time), len(pre_tIs)-1)

        t0 = self.hvars[pre_idx][0,0]
        tI = self.hvars[pre_idx][0,-1]
        tf = self.hvars[pre_idx+1][0,-1]

        xObjectI, dxObjectI, dxObjectI_post = self.compute_object_pre_post_state(pre_idx, tI)
        dxI_init = np.array([self.drvars[pre_idx][0,-1]/self.dhvars[pre_idx][0,-1],
                            self.drvars[pre_idx][1,-1]/self.dhvars[pre_idx][0,-1]])
        theta_init = np.arctan2(dxI_init[1], dxI_init[0]) + np.pi

        dxI = -(1/0.8)*(dxObjectI - dxObjectI_post)
        theta_close = np.arctan2(dxI[1], dxI[0]) + np.pi
        xI = xObjectI + 0.40*np.array([np.cos(theta_close), np.sin(theta_close)])
        xI_post = xI
        dxI_post = dxObjectI

        constraints = []

        # Derivative control points
        drvars, ddrvars = [], []
        for r in rvars:
            drvars_r, drv_constraints_r = get_derivative_control_points_cvxpy(r)
            drvars.append(drvars_r)
            constraints += drv_constraints_r

            ddrvars_r, ddrv_constraints_r = get_derivative_control_points_cvxpy(r, der_order=2)
            ddrvars.append(ddrvars_r)
            constraints += drv_constraints_r
        dhvars, ddhvars = [], []
        for h in hvars:
            dhvars_h, dhv_constraints_h = get_derivative_control_points_cvxpy(h)
            dhvars.append(dhvars_h)
            constraints += dhv_constraints_h

            ddhvars_h, ddhv_constraints_h = get_derivative_control_points_cvxpy(h, der_order=2)
            dhvars.append(ddhvars_h)
            constraints += ddhv_constraints_h        

        # Ensure increasing time
        for dh in dhvars:
            for i in range(dh.shape[1]):
                constraints.append(dh[0, i] >= 1e-1)

        # Continuity constraints
        constraints += [
            rvars[0][:, -1] == rvars[-1][:, 0],
            hvars[0][:, -1] == hvars[-1][:, 0],
        ]

        # Workspace bounds
        for rv in rvars:
            for i in range(rv.shape[1]):
                constraints += [
                    rv[0, i] >= 0.3,
                    rv[0, i] <= 3.7,
                    rv[1, i] >= -1.45,
                    rv[1, i] <= 1.45
                ]

        # Initial state
        x0 = self.robot_local_position[0:2]
        dx0 = self.robot_local_velocity[0:2]
        constraints += [
            rvars[0][:, 0] == x0,
            hvars[0][:, 0] == t0,
            drvars[0][:, 0] == dx0 * dhvars[0][:, 0]
        ]

        # Final state
        constraints += [
            rvars[-1][:, -1] == self.rvars[pre_idx+1][0:2, -1],
            hvars[-1][:, -1] == self.hvars[pre_idx+1][0, -1],
            drvars[-1][:, -1] == self.drvars[pre_idx+1][0:2, -1],
            dhvars[-1][:, -1] == self.dhvars[pre_idx+1][0, -1]
        ]

        # Impact condition
        constraints.append(hvars[0][:, -1] == tI)

        # Impact relaxation via delta
        delta = cp.Variable(nonneg=True)
        constraints += [
            rvars[0][:, -1] <= xI + delta,
            rvars[0][:, -1] >= xI - delta,
            rvars[-1][:, 0] <= xI_post + delta,
            rvars[-1][:, 0] >= xI_post - delta,
            drvars[0][:, -1] <= dxI * dhvars[0][:, -1] + delta,
            drvars[0][:, -1] >= dxI * dhvars[0][:, -1] - delta,
            drvars[-1][:, 0] <= dxI_post * dhvars[-1][:, 0] + delta,
            drvars[-1][:, 0] >= dxI_post * dhvars[-1][:, 0] - delta
        ]

        # Cost: quadratic in accelerations
        cost = 0
        for idx in range(len(ddhvars)):
            for i in range(ddhvars[idx].shape[1]):
                for d in range(2):
                    cost += cp.square(ddrvars[idx][d, i])
                cost += cp.square(ddhvars[idx][0, i])
        cost += 1e4 * delta

        # Solve QP
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, verbose=False)  # ECOS or OSQP for QP

        self.get_logger().info(f"CVXPY solve time: {time.time()-t_start:.3f}s")
        self.get_logger().info(f"delta: {delta.value:.3f}")

        # Extract solution
        rvars = [rvar.value for rvar in rvars]
        hvars = [hvar.value for hvar in hvars]
        # Pad rvars with an extra row of zeros (because there is orientation)
        rvars = [np.vstack([rvar, np.zeros((1, rvar.shape[1]))]) for rvar in rvars]
        # Construct replanned rvars, hvars, idvars, other_names
        self.re_rvars = self.rvars.copy()
        self.re_hvars = self.hvars.copy()
        self.re_idvars = self.idvars.copy()
        self.re_other_names = self.other_names.copy()
        self.re_rvars[pre_idx] = rvars[0]
        self.re_rvars[pre_idx + 1] = rvars[1]
        self.re_hvars[pre_idx] = hvars[0]
        self.re_hvars[pre_idx + 1] = hvars[1]
        # Write replanned data to CSV
        write_rvars = [rvars[0], rvars[1]]
        write_hvars = [hvars[0], hvars[1]]
        write_ids = ['pre', 'post']
        write_other_names = [self.object_ns[1:], self.object_ns[1:]]
        home_dir = os.path.expanduser("~")
        replans_path = os.path.join(home_dir, "space_ws", "replans")
        plan_to_csv(write_rvars, write_hvars, write_ids, write_other_names,
                    robot_name=f"{self.robot_name}_{self.Ncalls}",
                    scenario_name=self.scenario_name,
                    path=replans_path)
        self.Ncalls += 1
        self.get_logger().info(f"The whole ordeal took {time.time()-t_start:.3f}s")


def main(args=None):
    rclpy.init(args=args)
    replanner = RePlanner()
    rclpy.spin(replanner)

    replanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()