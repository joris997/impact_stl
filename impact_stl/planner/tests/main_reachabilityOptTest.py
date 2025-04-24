import gurobipy as gp
scs = {v: k for k, v in vars(gp.StatusConstClass).items() if k[0].isupper()}

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import pypolycontain as pp 
import time
import sys

sys.path.append('.')
from utilities.zonotopes import zonotope
from utilities.opt_helpers import createSupZonotope


A = np.array([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]])
X0 = pp.zonotope(x=np.array([0,0,0,0]), G=np.array([[0.25,0,0,0],
                                                    [0,0.25,0,0],
                                                    [0,0,0,0],
                                                    [0,0,0,0]]))

target = pp.zonotope(x=np.array([10,10,0,0]), G=np.array([[1,0,0,0],
                                                          [0,1,0,0],
                                                          [0,0,0,0],
                                                          [0,0,0,0]]))

T = 10
AIT = np.eye(4) + A*T



prog = gp.Model("reachability")
prog.setParam('OutputFlag', 0)

X0 = zonotope(x=X0.x, G=X0.G)

# get the outer vertices of the X0 zonotope
X0_vertices = []
cs = [prog.addMVar((2,), lb=0, ub=gp.GRB.INFINITY, name="asdf") for i in range(16)]
x0_vertices = [prog.addMVar((4,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY) for i in range(16)]
for i in range(2**4):
    # x0_i = X0.x + signs@X0.G          as G is a diagonal matrix (and will remain so!)
    signs = [(-1)**((i >> j) & 1) for j in range(4)]
    # Create the zonotope of the vertex
    X0_vertices.append( zonotope(x=x0_vertices[i],Gdiag=np.array([0,0,cs[i][0],cs[i][1]])) )

    try:
        prog.addConstrs((x0_vertices[i][j] == X0.x[j] + signs[j]*X0.Gdiag[j] for j in range(2)), name=f"x0_{i}_1")
    except Exception as e:
        print("x0_vertices error:", e)

# get the time propagation of the vertices of the X0 zonotope
Xf_vertices = []
xf_vertices = [prog.addMVar((4,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY) for i in range(16)]
for i in range(2**4):
    # xf_i = expAT@x0_i == (I + AT)x0_i         as A^2 = 0 for double integrator
    # for X0, we do this on x and G because it's a linear transformation
    x = prog.addMVar((4,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
    G = prog.addMVar((4,4), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
    Gabs = prog.addMVar((4,4), lb=0, ub=gp.GRB.INFINITY)
    Gdiag = prog.addMVar((4,), lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)
    try:
        # X0_vertices[i][0] = x, X0_vertices[i][1] = Gdiag
        prog.addConstr(x == AIT@X0_vertices[i].x, name=f"x_{i}")
        G_tmp = AIT@np.diag(X0_vertices[i].Gdiag)
        for jrow in range(4):
            for jcol in range(4):
                # G = AIT@X0_vertices[i]['G']
                prog.addConstr(G[jrow,jcol] == G_tmp[jrow,jcol], name=f"G_{i}_{jrow}_{jcol}")
                prog.addConstr(Gabs[jrow,jcol] == gp.abs_(G[jrow,jcol]), name=f"Gabs_{i}_{jrow}_{jcol}")
            
            prog.addConstr(Gdiag[jrow] == gp.quicksum([Gabs[jrow,jcol] for jcol in range(4)]), name=f"Gdiag_{i}_{jrow}")

        Xf_vertices.append( zonotope(x=x,Gdiag=Gdiag) )
    except Exception as e:
        print("Xf_vertices error:", e)

    # # for x0, we do this via the linear transformation on the vertex directly
    # try:
    #     prog.addConstr(xf_vertices[i] == AIT@x0_vertices[i], name=f"xf_{i}")
    # except Exception as e:
    #     print("xf_vertices error:", e)

# ensure that the xf_vertices are inside the target
for i in range(2**4):
    # Xf_vertices[i] \ in target
    try:
        # we need to ensure that the upper bound of Xf_vertices[i] is below the upper bound of target
        # and the lower bound of Xf_vertices[i] is above the lower bound of target  
        prog.addConstr(Xf_vertices[i].x[0] + Xf_vertices[i].Gdiag[0] <= target.x[0] + target.G[0,0], name=f"target_Xf1_{i}")
        prog.addConstr(Xf_vertices[i].x[0] - Xf_vertices[i].Gdiag[0] >= target.x[0] - target.G[0,0], name=f"target_Xf2_{i}")
        prog.addConstr(Xf_vertices[i].x[1] + Xf_vertices[i].Gdiag[1] <= target.x[1] + target.G[1,1], name=f"target_Yf1_{i}")
        prog.addConstr(Xf_vertices[i].x[1] - Xf_vertices[i].Gdiag[1] >= target.x[1] - target.G[1,1], name=f"target_Yf2_{i}")
    except Exception as e:
        print("target error:", e)

# now obtain the encompassing zonotope of all 16 vertex zonotope Xf_vertices[i]
# as a gurobipy variable with a diagonal matrix
Xf = createSupZonotope(prog, Xf_vertices)

# set the objective, maximize the minimum of cs
cs_mins = [prog.addMVar((1,), lb=0, ub=gp.GRB.INFINITY) for i in range(16)]
for i in range(16):
    prog.addConstr(cs_mins[i] == gp.min_([cs[i][j] for j in range(2)]))
cs_min = prog.addVar(lb=0, ub=gp.GRB.INFINITY)
prog.addConstr(cs_min == gp.min_([cs_mins[i] for i in range(16)]))
prog.setObjective(cs_min, gp.GRB.MAXIMIZE)


t0 = time.time()
prog.optimize()
print(f"Optimization took {time.time()-t0} seconds")
print(f"Code: {scs[prog.status]}")

if prog.status == gp.GRB.INFEASIBLE:
        prog.computeIIS()
        print("IIS:")
        for c in prog.getConstrs():
            if c.IISConstr: print(f'\t{c.constrname}: {prog.getRow(c)} {c.Sense} {c.RHS}')

# Print the results
print(f"obj: {prog.objVal}")
for i in range(16):
    print(f"velocities {i}: {x0_vertices[i].X}")

X0_sol = X0
Xf_sol = zonotope(x=Xf.x.X, Gdiag=Xf.Gdiag.X)

# Plot the results
P = np.array([[1,0,0,0],[0,1,0,0]])
fig, ax = plt.subplots()

for i in range(16):
    Xf_i = pp.zonotope(x=P@Xf_vertices[i].x.X, G=np.diag(P@Xf_vertices[i].Gdiag.X), color='b')
    try:
        pp.visualize([Xf_i], fig=fig, ax=ax, alpha=0.05)
    except:
        pass

X0_plot = pp.zonotope(x=P@X0_sol.x, G=P@np.diag(X0_sol.Gdiag), color='g')
Xf_plot = pp.zonotope(x=P@Xf_sol.x, G=P@np.diag(Xf_sol.Gdiag), color='r')
try:
    pp.visualize([X0_plot,Xf_plot], fig=fig, ax=ax, alpha=0.25)
except Exception as e:
    print(e)
    pass

ax.set_aspect('equal')
plt.show()
