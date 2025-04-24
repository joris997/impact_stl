import numpy as np
import scipy as sp
from scipy import special, optimize


def bernstein_poly(i,n,t):
    return special.comb(n, i) * ( t**i ) * (1 - t)**(n-i)

def eval_bezier(cps, N=100):
    # if cps has three dimensions we consider the parallel bezier
    # for reachability
    if len(cps.shape) == 3:
        vals = np.zeros((cps.shape[0],N,cps.shape[2]))
        for i in range(cps.shape[2]):
            vals[:,:,i] = eval_bezier(cps[:,:,i],N)
        return vals
    else:
        ndim = cps.shape[0]
        n = cps.shape[1]-1

        ts = np.linspace(0,1,N)
        vals = np.zeros((ndim,N))
        for i in range(n+1):
            vals += np.outer(bernstein_poly(i,n,ts),cps[:,i]).T
        return vals

def value_bezier(cps, t):
    if len(cps.shape) == 3:
        # if cps has three dimensions we consider the parallel beziers
        vals = np.zeros(cps.shape)
        for i in range(cps.shape[2]):
            vals[:,:,i] = value_bezier(cps[:,:,i],t)
    else:
        # otherwise we consider the bezier curve (cps.shape = [dim,order])
        ndim = cps.shape[0]
        n = cps.shape[1]-1

        vals = np.zeros(ndim)
        for i in range(n+1):
            vals += bernstein_poly(i,n,t)*cps[:,i]
        return vals

def eval_t(hvars, t):
    t_array = np.array([hvar[0,0] for hvar in hvars])
    idx = np.where(t_array <= t)[0][-1]

    if t > hvars[-1][0,-1] - 1e-3:
        s = 1
    else:
        error = lambda s: value_bezier(hvars[idx],s)[0] - t
        s = optimize.root_scalar(error, bracket=[0,1]).root
    return idx, s


def get_derivative_control_points_gurobi(control_points,der_order=1):
    # "control_points": [self.dim,self.order]
    # "der_order": positive integer
    assert(der_order > 0)
    dim = control_points.shape[0]
    order = control_points.shape[1]-1

    if order == 0:
        dcontrol_points = 0*control_points[:,0]
    else:
        dcontrol_points = 0*control_points[:,0:order]
        for i in range(order):
            dcontrol_points[:,i] = order*(control_points[:,i+1]-control_points[:,i])
    
    # use recursion for higher order derivatives
    if der_order > 1:
        return get_derivative_control_points_gurobi(dcontrol_points,der_order-1)
    else:
        return dcontrol_points
    
def get_derivative_control_points_parallel_gurobi(control_points,der_order=1):
    assert(der_order > 0)
    dim = control_points.shape[0]
    order = control_points.shape[1]-1
    nparallel = control_points.shape[2]

    if order == 0:
        dcontrol_points = 0*control_points[:,0,:]
    else:
        dcontrol_points = 0*control_points[:,0:order,:]
        for i in range(order):
            dcontrol_points[:,i,:] = order*(control_points[:,i+1,:]-control_points[:,i,:])
        
    if der_order > 1:
        return get_derivative_control_points_parallel_gurobi(dcontrol_points,der_order-1)
    else:
        return dcontrol_points