import casadi as cs
from impact_stl.helpers.beziers import get_derivative_control_points_gurobi
import numpy as np
import time

def solve_a_b_plan(x0,dx0,xf,dxf,dt):
    ocp = cs.Opti()
    rvars = [ocp.variable(2,4) for _ in range(2)]
    hvars = [ocp.variable(1,4) for _ in range(2)]

    t0 = 0.0
    tf = t0 + dt

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
    ocp.subject_to(drvars[0][:,-1] == drvars[-1][:,0])
    ocp.subject_to(dhvars[0][:,-1] == dhvars[-1][:,0])

    # initial state
    ocp.subject_to(rvars[0][:,0] == x0)
    ocp.subject_to(hvars[0][:,0] == t0)
    ocp.subject_to(drvars[0][:,0] == dx0*dhvars[0][:,0])
    # final state
    ocp.subject_to(rvars[-1][:,-1] == xf)
    ocp.subject_to(hvars[-1][:,-1] == tf)
    ocp.subject_to(drvars[-1][:,-1] == dxf*dhvars[-1][:,-1])

    print("added all casadi constraints")

    # cost
    cost = 0
    for idx in range(len(ddhvars)):
        for i in range(ddhvars[idx].shape[1]):
            for d in range(2):
                cost += ddrvars[idx][d,i]*ddrvars[idx][d,i]
            cost += ddhvars[idx][0,i]*ddhvars[idx][0,i]
    ocp.minimize(cost)
    print("added casadi cost")

    # set solver method
    opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
    ocp.solver('ipopt',opts)

    # solve
    t0 = time.time()
    sol = ocp.solve()
    print(f"Solving took {time.time()-t0} seconds")

    rvars = [sol.value(rvar).reshape((2,4)) for rvar in rvars]
    hvars = [sol.value(hvar).reshape((1,4)) for hvar in hvars]
    # pad rvars with an extra row of zeros (because there is orientation)
    rvars = [np.vstack([rvar,np.zeros((1,rvar.shape[1]))]) for rvar in rvars]
    idvars = ['none','none']

    return rvars,hvars,idvars