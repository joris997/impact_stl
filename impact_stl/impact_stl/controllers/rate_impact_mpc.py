#!/usr/bin/env python
__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import numpy as np
import casadi as cs
import time

from impact_stl.models.spacecraft_rate_model import SpacecraftRateModel

class SpacecraftRateImpactMPC():
    def __init__(self, model, Tf=1.0, N=10):
        self.model = model
        self.Tf = Tf
        # we add 1 extra timestep which ends up being the instantaneous
        # state change due to the impact
        self.N = N+1
        # the duration of the timesteps is with the old N to keep the 
        # overall duration consistent with the non-impact mpc
        self.dt = self.Tf/N

        self.nx = 10
        self.nu = 6

        self.mpcs = []
        self.params = {}
        self.vars = {}

        # cost matrices [sitl]
        self.Q = np.diag([1e1, 1e1, 0, 8e3, 8e3, 0, 8e3])
        self.Q_e = 10 * self.Q
        self.R = 2*np.diag([1e-2, 1e-2, 1e-2, 2e0, 2e0, 2e0])
        
        # # cost matrices [hw]
        # self.Q = np.diag([5e1, 5e1, 5e1, 25, 25, 25, 8e3])
        # self.Q_e = 10 * self.Q
        # self.R = 2*np.diag([1e-2, 1e-2, 1e-2, 2e0, 2e0, 2e0])

        for impact_idx in range(0,self.N):
            ocp,vars,params = self.setup(impact_idx)
            mpc = {}
            mpc['ocp'], mpc['vars'], mpc['params'] = ocp, vars, params
            self.mpcs.append(mpc)

    def setup(self,impact_idx=0):
        # create casadi optimization problem
        ocp = cs.Opti()
        X = ocp.variable(self.nx,self.N+1)
        U = ocp.variable(self.nu,self.N)

        x0 = ocp.parameter(self.nx)
        xref = ocp.parameter(self.nx,self.N+1)
        xpre = ocp.parameter(self.nx)
        xpost = ocp.parameter(self.nx)
        dts = ocp.parameter(self.N)

        delta_pre = ocp.variable(1)
        delta_post = ocp.variable(1)

        # set initial state
        ocp.subject_to(X[:,0] == x0)

        # set dynamics constraints
        for i in range(self.N):
            if i == impact_idx:
                ocp.subject_to(cs.sumsqr(X[:,i]-xpre) <= delta_pre)
                # we don't care much about the post-impact velocity
                ocp.subject_to(cs.sumsqr(X[:,i+1]-xpost) <= delta_post)

                # to stay true to the N, N+1 format, we just don't allow control
                # during the instantaneous impact duration
                ocp.subject_to(U[:,i] == np.zeros((self.nu,)))
            else:
                ocp.subject_to(self.model.get_casadi_ode(X[:,i],U[:,i],dts[i]) == X[:,i+1])
                # ocp.subject_to(self.model.get_casadi_rk4(X[:,i],U[:,i],dts[i]) == X[:,i+1])

        # control input constraints
        for i in range(self.N):
            ocp.subject_to(self.model.u_lb <= U[:,i])
            ocp.subject_to(U[:,i] <= self.model.u_ub)

        # set cost
        Q = ocp.parameter(self.nx-3,self.nx-3)
        Q_e = ocp.parameter(self.nx-3,self.nx-3)
        R = ocp.parameter(self.nu,self.nu)
        cost_eq = 0
        for i in range(impact_idx): #range(self.N):
            cost_eq += self.calculate_state_error(X[:,i], xref[:,i], Q)
            cost_eq += U[:,i].T @ R @ U[:,i]
        cost_eq += self.calculate_state_error(X[:,-1], xref[:,-1], Q_e)
        # for i in range(self.N):
        #     if i >= impact_idx:
        #         cost_eq += self.calculate_state_error(X[:,i], xref[:,i], 0.1*Q)
        #         cost_eq += U[:,i].T @ (0.1*R) @ U[:,i]
        #     else:
        #         cost_eq += self.calculate_state_error(X[:,i], xref[:,i], Q)
        #         cost_eq += U[:,i].T @ R @ U[:,i]
        # cost_eq += self.calculate_state_error(X[:,-1], xref[:,-1], 0.1*Q_e)
        ocp.minimize(cost_eq + 100*delta_pre + delta_post)

        # set solver method
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        ocp.solver('ipopt',opts)

        # set consistent variables and parameters
        params, vars = {}, {}
        params['x0'] = x0
        params['xref'] = xref
        params['xpre'] = xpre
        params['xpost'] = xpost
        params['dts'] = dts
        # costs
        params['Q'] = Q
        params['Q_e'] = Q_e
        params['R'] = R

        vars['X'] = X
        vars['U'] = U
        vars['delta_pre'] = delta_pre
        vars['delta_post'] = delta_post

        return ocp,vars,params
    
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

    def solve(self, x0, xpre, xpost, impact_idx=0, setpoints=None, times=None, 
              weights={'Q':None, 'Q_e':None, 'R':None},
              initial_guess={'X':None, 'U':None},
              verbose=False):
        print("attempting to solve impact_mpc")
        print(f"- impact_idx: {impact_idx}")
        print(f"- times: {times}")
        # impact_idx: the time-step of the mpc indicating the pre-impact state
        t0 = time.time()
        idx = impact_idx

        # print(f"X-shape: {self.mpcs[idx]['vars']['X'].shape}")
        # print(f"U-shape: {self.mpcs[idx]['vars']['U'].shape}")
    
        # set initial guess
        if initial_guess['X'] is not None:
            self.mpcs[idx]['ocp'].set_initial(self.mpcs[idx]['vars']['X'], initial_guess['X'])
        if initial_guess['U'] is not None:
            self.mpcs[idx]['ocp'].set_initial(self.mpcs[idx]['vars']['U'], initial_guess['U'])

        # set x0 parameter
        self.mpcs[idx]['ocp'].set_value(self.mpcs[idx]['params']['x0'], x0)
        # set setpoints parameter
        xref = np.hstack(setpoints)
        self.mpcs[idx]['ocp'].set_value(self.mpcs[idx]['params']['xref'], xref)
        # set dts parameter
        if times is None:
            dts = [self.dt for _ in range(self.N)]
        else:
            dts = [times[i+1]-times[i] for i in range(len(times)-1)]
        # print(f"dts: {dts}") if verbose else None
        self.mpcs[idx]['ocp'].set_value(self.mpcs[idx]['params']['dts'],dts)
        # set pre and post impact states
        self.mpcs[idx]['ocp'].set_value(self.mpcs[idx]['params']['xpre'],xpre)
        self.mpcs[idx]['ocp'].set_value(self.mpcs[idx]['params']['xpost'],xpost)

        # set cost matrices
        Q = self.Q if weights['Q'] is None else weights['Q']
        Q_e = self.Q_e if weights['Q_e'] is None else weights['Q_e']
        R = self.R if weights['R'] is None else weights['R']
        self.mpcs[idx]['ocp'].set_value(self.mpcs[idx]['params']['Q'], Q)
        self.mpcs[idx]['ocp'].set_value(self.mpcs[idx]['params']['Q_e'], Q_e)
        self.mpcs[idx]['ocp'].set_value(self.mpcs[idx]['params']['R'], R)

        # solve the mpc
        sol = self.mpcs[idx]['ocp'].solve()

        X_pred = sol.value(self.mpcs[idx]['vars']['X'])
        U_pred = sol.value(self.mpcs[idx]['vars']['U'])

        print(f"Optimization time: {time.time()-t0}") if verbose else None
        print(f"pre norm diff:  {sol.value(self.mpcs[idx]['vars']['delta_pre'])}") if verbose else None
        print(f"post norm diff: {sol.value(self.mpcs[idx]['vars']['delta_post'])}") if verbose else None

        return X_pred, U_pred
