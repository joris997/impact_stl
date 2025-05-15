#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import numpy as np
import casadi as cs
import time

from impact_stl.models.spacecraft_rate_model import SpacecraftRateModel

class SpacecraftRateMPC():
    def __init__(self, model, Tf=1.0, N=10, add_cbf=False):
        self.model = model
        self.Tf = Tf
        self.N = N
        self.dt = self.Tf/self.N

        self.add_cbf = add_cbf

        self.nx = 10
        self.nu = 6

        self.params = {}
        self.vars = {}

        # cost matrices (SITL)
        # self.Q = np.diag([10e0, 10e0, 10e0, 10e-1, 10e-1, 10e-1, 8e-1])
        # self.Q_e = 10 * self.Q
        # self.R = 2*np.diag([1e-2] * 6)

        # cost matrices (HW)
        # self.Q = np.diag([10e1, 10e1, 10e1, 10e-1, 10e-1, 10e-1, 8e0])
        # self.Q_e = 10 * self.Q
        # self.R = np.diag([1e-3] * 6)

        # px4-mpc 
        self.Q = np.diag([5e0, 5e0, 5e0, 8e-1, 8e-1, 8e-1, 8e3])
        self.Q_e = 10 * self.Q
        self.R = 2*np.diag([1e-2, 1e-2, 1e-2, 2e0, 2e0, 2e0])

        if self.add_cbf:
            p_r = cs.SX.sym('p_r', 3)
            v_r = cs.SX.sym('v_r', 3)
            q_r = cs.SX.sym('q_r', 4)
            u_r = cs.SX.sym('u_r', self.nu)
            p_o = cs.SX.sym('p_o', 3)
            v_o = cs.SX.sym('v_o', 3)
            q_o = cs.SX.sym('q_o', 4)
            u_o = cs.SX.sym('u_o', self.nu)
            h = cs.sumsqr(p_r[0:2] - p_o[0:2]) - (0.2+0.2+0.1)**2 # 2 times the radius of the object + 0.1 m
            x = cs.vertcat(p_r,p_o)
            dx = cs.vertcat(v_r,v_o)

            X_r = cs.vertcat(p_r,v_r,q_r)
            X_o = cs.vertcat(p_o,v_o,q_o)
            U_r = u_r
            U_o = u_o

            dh = cs.jacobian(h,x) @ dx
            ddh = cs.jacobian(dh,x) @ dx + cs.jacobian(dh,dx) @ cs.vertcat(self.model.get_casadi_dx(X_r,u_r)[3:6],
                                                                           self.model.get_casadi_dx(X_o,u_o)[3:6])
            print(f"h: {h}")
            print(f"dh: {dh}")
            # print(f"jac(h,x_r): {cs.jacobian(h,x_r)}")
            # print(f"dh_dx_r*dx: {cs.jacobian(h,x_r)@self.model.get_casadi_dx(x_r,u_r)}")
            # print(f"d(dh_dx_r*dx)/dx: {cs.jacobian(cs.jacobian(h,x_r)@self.model.get_casadi_dx(x_r,u_r),x_r)}")
            print(f"ddh: {ddh}")

            # now create functions out of these
            self.h = cs.Function('h', [X_r,X_o], [h])
            self.dh = cs.Function('dh', [X_r,X_o], [dh])
            self.ddh = cs.Function('ddh', [X_r,X_o,U_r,U_o], [ddh])
            self.beta = 1e0 # h
            self.alpha  = 2e0 # dh
            # the barrier constraint: ddh + alpha*dh + beta*h >= 0

        self.ocp = self.setup()

    def setup(self):
        # create casadi optimization problem
        ocp = cs.Opti()
        X = ocp.variable(self.nx,self.N+1)
        U = ocp.variable(self.nu,self.N)

        x0 = ocp.parameter(self.nx)
        xref = ocp.parameter(self.nx,self.N+1)

        # set initial state
        ocp.subject_to(X[:,0] == x0)

        # set dynamics constraints
        for i in range(self.N):
            ocp.subject_to(self.model.get_casadi_ode(X[:,i],U[:,i],self.dt) == X[:,i+1])
            # ocp.subject_to(self.model.get_casadi_rk4(X[:,i],U[:,i],self.dt) == X[:,i+1])

        # control input constraints
        for i in range(self.N):
            ocp.subject_to(self.model.u_lb <= U[:,i])
            ocp.subject_to(U[:,i] <= self.model.u_ub)

        # potential cbf constraints
        if self.add_cbf:
            X_r = X[:,0]
            U_r = U[:,0]
            X_o = ocp.parameter(self.nx)
            U_o = ocp.parameter(self.nu)
            delta = ocp.variable(1)
            OffSwitch = ocp.parameter(1)
            ocp.subject_to(self.ddh(X_r,X_o,U_r,U_o) + self.alpha*self.dh(X_r,X_o) + \
                           self.beta*self.h(X_r,X_o) >= -delta - OffSwitch)
            ocp.subject_to(delta >= 0)
            self.params['X_o'] = X_o
            self.params['U_o'] = U_o
            self.params['OffSwitch'] = OffSwitch
            self.vars['delta'] = delta
            

        # set cost
        Q = ocp.parameter(self.nx-3,self.nx-3)
        Q_e = ocp.parameter(self.nx-3,self.nx-3)
        R = ocp.parameter(self.nu,self.nu)
        cost_eq = 0
        for i in range(self.N):
            cost_eq += self.calculate_state_error(X[:,i], xref[:,i], Q)
            cost_eq += U[:,i].T @ R @ U[:,i]
        cost_eq += self.calculate_state_error(X[:,-1], xref[:,-1], Q_e)

        if self.add_cbf:
            cost_eq += 100*self.vars['delta']

        ocp.minimize(cost_eq)

        # set solver method
        # # IPOPT
        # # IPOPT:
        ipopt_opts = {
            'ipopt.print_level': 0,
            'ipopt.warm_start_bound_push': 1e-4,
            'ipopt.warm_start_bound_frac': 1e-4,
            'ipopt.warm_start_slack_bound_frac': 1e-4,
            'ipopt.warm_start_slack_bound_push': 1e-4,
            'ipopt.warm_start_mult_bound_push': 1e-4,
            'print_time': False,
            'verbose': False,
        }
        ocp.solver('ipopt',ipopt_opts)

        # SQP:
        # qp_opts = {
        #     'max_iter': 10,
        #     'error_on_fail': False,
        #     'print_header': False,
        #     'print_iter': False
        # }
        # sqp_opts = {
        #     'max_iter': 100,
        #     'qpsol': 'qrqp',
        #     'convexify_margin': 1e-5,
        #     'print_header': False,
        #     'print_time': False,
        #     'print_iteration': False,
        #     'qpsol_options': qp_opts
        # }
        # ocp.solver('sqpmethod',sqp_opts)

        # set consistent variables and parameters
        self.params['x0'] = x0
        self.params['xref'] = xref
        self.params['Q'] = Q
        self.params['Q_e'] = Q_e
        self.params['R'] = R

        self.vars['X'] = X
        self.vars['U'] = U

        return ocp
    
    def calculate_state_error(self, x, xref, Q):
        # state: p, v, q
        es = x - xref
        es = es[0:6]
        cost_es = es.T @ Q[0:6,0:6] @ es

        # quaternion cost
        q = x[6:10].reshape((4,1))
        qref = xref[6:10].reshape((4,1))
        eq = 1 - (q.T @ qref)**2 
        cost_eq = eq.T @ Q[6,6].reshape((1, 1)) @ eq

        return cost_eq + cost_es

    def solve(self, x0, setpoints=None,
              weights={'Q': None, 'Q_e': None, 'R': None},
              initial_guess={'X': None, 'U': None},
              xobj=None, enable_cbf=True,
              logger=None, verbose=False):
        t0 = time.time()

        # print(f"x0: {x0}")

        # set initial guess if we are getting any
        if initial_guess['X'] is not None:
            self.ocp.set_initial(self.vars['X'], initial_guess['X'])
        if initial_guess['U'] is not None:
            self.ocp.set_initial(self.vars['U'], initial_guess['U'])

        # set x0 parameter
        self.ocp.set_value(self.params['x0'], x0)

        # set setpoints parameter
        xref = np.hstack(setpoints)
        self.ocp.set_value(self.params['xref'], xref)

        # set cost matrices if we are getting any
        Q = self.Q if weights['Q'] is None else weights['Q']
        Q_e = self.Q_e if weights['Q_e'] is None else weights['Q_e']
        R = self.R if weights['R'] is None else weights['R']
        self.ocp.set_value(self.params['Q'], Q)
        self.ocp.set_value(self.params['Q_e'], Q_e)
        self.ocp.set_value(self.params['R'], R)

        # set other object parameters if we should add the cbf (otherwise these
        # parameters do not exist)
        if xobj is not None and self.add_cbf:
            xobj_old = xobj.copy()
            #! change between hw and sitl
            # xobj[0], xobj[1] = -xobj_old[1], xobj_old[0]
            self.ocp.set_value(self.params['X_o'], xobj)
            self.ocp.set_value(self.params['U_o'], np.zeros((self.nu,1)))
            if enable_cbf:
                self.ocp.set_value(self.params['OffSwitch'], 0)
            else:
                self.ocp.set_value(self.params['OffSwitch'], 10000) # make it a trivial constraint
            print(f"h: {self.h(x0,xobj)}")
            print(f"dh: {self.dh(x0,xobj)}")
            print(f"ddh: {self.ddh(x0,xobj,np.zeros((self.nu,1)),np.zeros((self.nu,1)))}")

        try:
            sol = self.ocp.solve()
            X_pred = sol.value(self.vars['X'])
            U_pred = sol.value(self.vars['U'])
        except Exception as e:
            print(f"Optimization failed: {e}")
            X_pred = np.zeros((self.nx, self.N+1))
            U_pred = np.zeros((self.nu, self.N))

        logger.info(f"error: {self.calculate_state_error(X_pred[:,0], xref[:,0], Q)}") if verbose and logger is not None else None
        logger.info(f"X_pred:  {X_pred[:,0]}") if verbose and logger is not None else None
        logger.info(f"Optimization time: {time.time()-t0}") if verbose and logger is not None else None
        return X_pred, U_pred


#! Impact detector with direction information
# __author__ = "Joris Verhagen"
# __contact__ = "jorisv@kth.se"
# import numpy as np
# import rclpy
# from rclpy.clock import Clock
# from rclpy.node import Node
# from push_stl.helpers.qos_profiles import RELIABLE_QOS, NORMAL_QOS
# from px4_msgs.msg import VehicleLocalPosition
# from my_msgs.msg import StampedBool
# class ImpactDetector(Node):
#     def __init__(self):
#         super().__init__('impact_detector')
#         self.threshold = self.declare_parameter('threshold', 1.0).value # 1.0 for sim, 3.0 for hw
#         self.gz = self.declare_parameter('gz', True).value
#         if self.gz:
#             self.local_position_sub = self.create_subscription(
#                 VehicleLocalPosition,
#                 'fmu/out/vehicle_local_position_gz',
#                 self.vehicle_local_position_callback,
#                 NORMAL_QOS)
#         else:
#             self.local_position_sub = self.create_subscription(
#                 VehicleLocalPosition,
#                 'fmu/out/vehicle_local_position',
#                 self.vehicle_local_position_callback,
#                 NORMAL_QOS)
#         self.publisher_impact = self.create_publisher(StampedBool, 'push_stl/impact_detected', RELIABLE_QOS)
#         self.publisher_impact_direction = self.create_publisher(StampedBool, 'push_stl/impact_direction', RELIABLE_QOS)
#         self.vehicle_acceleration = np.array([0.0, 0.0, 0.0])
#         self.vehicle_past_accelerations = np.zeros((3,))
#         self.past_accel_x = np.zeros((3,))
#         self.past_accel_y = np.zeros((3,))
#         # get the current time
#         self.t_start = Clock().now().nanoseconds/1000
#         self.t_wait  = 3
#         self.get_logger().info('Created an impact detector')
#     def vehicle_local_position_callback(self, msg):
#         if (Clock().now().nanoseconds/1000 - self.t_start)/1e6 < self.t_wait:
#             return
#         # TODO: handle NED->ENU transformation
#         self.vehicle_acceleration[0] = msg.ax
#         self.vehicle_acceleration[1] = -msg.ay
#         self.vehicle_acceleration[2] = -msg.az
#         self.vehicle_past_accelerations[:-1] = self.vehicle_past_accelerations[1:]
#         self.vehicle_past_accelerations[-1] = np.linalg.norm(self.vehicle_acceleration)
#         self.past_accel_x[:-1] = self.past_accel_x[1:]
#         self.past_accel_x[-1] = self.vehicle_acceleration[0]
#         self.past_accel_y[:-1] = self.past_accel_y[1:]
#         self.past_accel_y[-1] = self.vehicle_acceleration[1]
#         if np.mean(self.vehicle_past_accelerations) > self.threshold:
#             self.get_logger().info('--- IMPACT DETECTED ---')
#             msg = StampedBool()
#             msg.timestamp = int(Clock().now().nanoseconds / 1000)
#             msg.data[0] = True
#             if abs(np.mean(self.past_accel_x)) > abs(np.mean(self.past_accel_y)):
#                 self.get_logger().info('Impact direction: y')
#                 msg.data[1] = True
#             else:
#                 self.get_logger().info('Impact direction: x')
#                 msg.data[1] = False
#             self.publisher_impact.publish(msg)
#             self.get_logger().info('Impact message published')
# def main(args=None):
#     rclpy.init(args=args)
#     impact_detector = ImpactDetector()
#     rclpy.spin(impact_detector)
#     impact_detector.destroy_node()
#     rclpy.shutdown()
# if __name__ == '__main__':
#     main()