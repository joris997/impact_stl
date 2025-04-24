import gurobipy as gp
from utilities.zonotopes import zonotope
import numpy as np

# this file aims to provide helper functions for the optimization problems
# such that the main code is more readable

def createSupZonotope(prog, Zs, Zg=None):
    n = len(Zs)         # number of zonotopes to overapproximate
    dim = Zs[0].x.shape[0]  # dimension of the zonotopes

    if Zg is None:
        Z_x = prog.addMVar((dim,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
        Z_G = prog.addMVar((dim,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
        Zg = zonotope(x=Z_x, Gdiag=Z_G)

    Z_x_min = prog.addMVar((dim,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
    Z_x_max = prog.addMVar((dim,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)

    Z_vertices_x_max = prog.addMVar((dim,n), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
    Z_vertices_x_min = prog.addMVar((dim,n), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)

    for i in range(n):
        try:
            prog.addConstrs((Z_vertices_x_max[j,i] == Zs[i].x[j] + Zs[i].Gdiag[j] for j in range(dim)))
            prog.addConstrs((Z_vertices_x_min[j,i] == Zs[i].x[j] - Zs[i].Gdiag[j] for j in range(dim)))
        except Exception as e:
            print(f"Z_vertices max min error {i} : {e}")
        
    for i in range(dim):
        try:
            prog.addConstr(Z_x_max[i] == gp.max_([Z_vertices_x_max[i,j] for j in range(n)]))
            prog.addConstr(Z_x_min[i] == gp.min_([Z_vertices_x_min[i,j] for j in range(n)]))
        except Exception as e:
            print(f"Z_x_max min error {i}: {e}")

    for i in range(dim):
        try:
            prog.addConstr(Zg.x[i] == (Z_x_max[i] + Z_x_min[i])/2)
            prog.addConstr(Zg.Gdiag[i] == (Z_x_max[i] - Z_x_min[i])/2)
        except Exception as e:
            print(f"Z_x G error {i}: {e}")
        
    return Zg

def createBoundingZonotope(prog, curve, Zg=None):
    # given a bezier curve, create the bounding box (hyperrectangle) of the curve
    # as a zonotope
    ndim = curve.shape[0]
    ncp = curve.shape[1]

    if Zg is None:
        Z_x = prog.addMVar((ndim,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
        Z_G = prog.addMVar((ndim,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
        Zg = zonotope(x=Z_x, Gdiag=Z_G)

    max_dim = prog.addMVar((ndim,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
    min_dim = prog.addMVar((ndim,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
    for d in range(ndim):
        try:
            prog.addConstr(max_dim[d] == gp.max_(curve[d,cpi] for cpi in range(ncp)))
            prog.addConstr(min_dim[d] == gp.min_(curve[d,cpi] for cpi in range(ncp)))
        except Exception as e:
            print(f"max min dim error {d}: {e}")
    
    for d in range(ndim):
        try:
            prog.addConstr(Zg.x[d] == (max_dim[d] + min_dim[d])/2)
            prog.addConstr(Zg.Gdiag[d] == (max_dim[d] - min_dim[d])/2)
        except Exception as e:
            print(f"Z_x G error {d}: {e}")


def addBilinearConstr(prog, x2, y, x1):
    # create a constraint of the form x2 = y*x1
    # y might be either a binary variable or an integer variable
    # x1 might be either a positive real variable or a real variable
    x1_lb = x1.lb
    x1_ub = x1.ub
    y_lb = y.lb
    y_ub = y.ub

    if x1_lb >= 0:
        # This is the straightforward way!
        s = prog.addVar(lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
        prog.addConstr(s >= y_lb*x1, name="s_lb")
        prog.addConstr(s <= y_ub*x1, name="s_ub")
        prog.addConstr(s >= y*x1_lb, name="s_lb2")
        prog.addConstr(s <= y*x1_ub, name="s_ub2")
        prog.addConstr(x2 == s, name="x2_split")

    else:
        # This is the less straightfoward way.
        # we need to separate x1 into a positive and negative part
        x_1p = prog.addVar(lb=0, ub=gp.GRB.INFINITY, name="x_1p")
        x_1m = prog.addVar(lb=0, ub=gp.GRB.INFINITY, name="x_1m")
        s_p = prog.addVar(lb=0, ub=gp.GRB.INFINITY, name="s_p")
        s_m = prog.addVar(lb=0, ub=gp.GRB.INFINITY, name="s_m")
        b = prog.addVar(vtype=gp.GRB.BINARY, name="b")

        prog.addConstr(x1 == x_1p - x_1m, name="x1_split")
        prog.addConstr(x_1p <= x1_ub*b, name="x1p_ub")
        prog.addConstr(x_1m <= (1-b)*x1_ub, name="x1p_ub2")

        # now we can write two constraints for positive x_1p and x_1m
        prog.addConstr(s_p >= y_lb*x_1p, name="s_p_lb")
        prog.addConstr(s_p <= y_ub*x_1p, name="s_p_ub")
        prog.addConstr(s_p >= y*x1_lb, name="s_p_lb2")
        prog.addConstr(s_p <= y*x1_ub, name="s_p_ub2") 

        prog.addConstr(s_m >= y_lb*x_1m, name="s_m_lb")
        prog.addConstr(s_m <= y_ub*x_1m, name="s_m_ub")
        prog.addConstr(s_m >= y*x1_lb, name="s_m_lb2")
        prog.addConstr(s_m <= y*x1_ub, name="s_m_ub2")

        prog.addConstr(x2 == s_p - s_m, name="x2_split")

def constrainZinZ(prog,Zin,Zout):
    # Constrain that Zin is inside Zout
    # we're only concerned with hyperrectangles so we can check each dimension separately
    dim = Zin.x.shape[0]
    for i in range(dim):
        try:
            prog.addConstr(Zin.x[i] + Zin.Gdiag[i] <= Zout.x[i] + Zout.Gdiag[i], name=f"ZinZout{i}_1")
            prog.addConstr(Zin.x[i] - Zin.Gdiag[i] >= Zout.x[i] - Zout.Gdiag[i], name=f"ZinZout{i}_2")
        except Exception as e:
            print(f"constrainZinZ error {i}: {e}")

def constrainZoutZ(prog,Zin,Zout,bigM=1e4,condition=1):
    # Constrain that Zout is outside of Zin, with bigM constraints
    # with an external condition that can be either 0 or 1 (or a binary variable)
    # if cond==1, the stay-out constraint is enforced
    # we're only concerned with hyperrectangles so we can check each dimension separately
    dim = Zin.x.shape[0]
    zvar = prog.addMVar((2,2),vtype=gp.GRB.BINARY)
    for i in range(zvar.shape[0]):
        # Zin[i] left of Zout[i]
        prog.addConstr(Zin.x[i] + Zin.Gdiag[i] <= Zout.x[i] - Zout.Gdiag[i] + bigM*(1-zvar[i,0]))
        # Zin[i] right of Zout[i]
        prog.addConstr(Zin.x[i] - Zin.Gdiag[i] >= Zout.x[i] + Zout.Gdiag[i] - bigM*(1-zvar[i,1]))

    # constrain that sum of zvar >= 1
    prog.addConstr(gp.quicksum([zvar[i,j] for i in range(zvar.shape[0]) for j in range(zvar.shape[1])]) >= condition)

def constrainZoutH(prog,Zout,H,b,bigM=1e4,condition=1):
    # Constrain that the zonotope Zout is outside of the polygon 
    # defined by the inequality Hx <= b
    # if cond==1, the stay-out constraint is enforced
    # we're only concerned with hyperrectangles so we can check each dimension separately
    dim = H.shape[1]
    nfaces = H.shape[0]
    zvar = prog.addMVar((nfaces,),vtype=gp.GRB.BINARY)

    signs = np.array([[1,1],
                      [1,-1],
                      [-1,1],
                      [-1,-1]])

    for face in range(nfaces):
        # we check whether all outer vertices of the zonotope in x-y lay outside the halfspace
        for v in range(signs.shape[0]):
            try:
                ineqs = H@(Zout.x[0:2] + Zout.Gdiag[0:2]*signs[v,:])
                c = -ineqs[face] + b[face]
                prog.addConstr(c <= bigM*(1-zvar[face]))
            except Exception as e:
                print(f"constrainZoutH error {face},{v}: {e}")

    # constrain that sum of zvar >= 1
    prog.addConstr(gp.quicksum([z for z in zvar]) >= condition)

def constrainZinH(prog,Zin,H,b,bigM=1e4,condition=1):
    # Constrain that the zonotope Zin is inside the polygon 
    # defined by the inequality Hx <= b
    # we're only concerned with hyperrectangles so we can check each dimension separately
    dim = H.shape[1]
    nfaces = H.shape[0]

    signs = np.array([[1,1],
                      [1,-1],
                      [-1,1],
                      [-1,-1]])

    for face in range(nfaces):
        # we check whether all outer vertices of the zonotope in x-y lay outside the halfspace
        for v in range(2**(dim)):
            try:
                ineqs = H@(Zin.x[0:2] + Zin.Gdiag[0:2]@signs[v,:])
                c = -ineqs[face] + b[face]
                prog.addConstr(c >= -bigM*(1-condition))
            except Exception as e:
                print(f"constrainZinH error {face},{v}: {e}")

def constrainRinH(prog,rvars,H,b,bigM=1e4,condition=1):
    # Constrain that the rectangle rvars is inside the polygon 
    # defined by the inequality Hx <= b
    # we're only concerned with hyperrectangles so we can check each dimension separately
    dim = H.shape[1]
    nfaces = H.shape[0]

    for face in range(nfaces):
        # we check whether all outer vertices of the zonotope in x-y lay outside the halfspace
        for cp in range(rvars.shape[1]):
            try:
                ineqs = H@rvars[:,cp]
                c = -ineqs[face] + b[face]
                prog.addConstr(c >= -bigM*(1-condition))
            except Exception as e:
                print(f"constrainRinH error {face},{cp}: {e}")

def constrainRoutH(prog,rvars,H,b,bigM=1e4,condition=1):
    # Constrain that the rectangle rvars is outside the polygon 
    # defined by the inequality Hx <= b
    # we're only concerned with hyperrectangles so we can check each dimension separately
    dim = H.shape[1]
    nfaces = H.shape[0]
    zvar = prog.addMVar((nfaces,),vtype=gp.GRB.BINARY)

    for face in range(nfaces):
        # we check whether all outer vertices of the zonotope in x-y lay outside the halfspace
        for cp in range(rvars.shape[1]):
            try:
                ineqs = H@rvars[:,cp]
                c = -ineqs[face] + b[face]
                prog.addConstr(c <= bigM*(1-zvar[face]))
            except Exception as e:
                print(f"constrainRoutH error {face},{cp}: {e}")
    
    # constrain that sum of zvar >= 1
    prog.addConstr(gp.quicksum([z for z in zvar]) >= condition)

