import gurobipy as gp
scs = {v: k for k, v in vars(gp.StatusConstClass).items() if k[0].isupper()}

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import time

# we want to test if we can formulate a constraint x_2 = y x_1 in a MILP fasion
# This works for x_1 \in R_{\geq 0}, but does it work for x_1 \in R?

# the solution is based on an answer here
# https://or.stackexchange.com/questions/6028/mixed-integer-optimization-with-bilinear-constraint
# but adapted to take an integer y into account instead of a binary y as well as 
# x_1 \in R in stead of x_1 \in R_{\geq 0}

# create the model
m = gp.Model("robustness")
m.setParam('OutputFlag', 0)
m.setParam('NonConvex', 0)

# create the variables
x1_ub = 100
x1_lb = -100
x1 = m.addVar(lb=x1_lb, ub=x1_ub, name="x1")
x2 = m.addVar(lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY, name="x2")
# create integer y
y_lb = 0
y_ub = 10
y = m.addVar(vtype=gp.GRB.INTEGER, lb=y_lb, ub=y_ub, name="y")

# #################################################
# # create the constraint the direct way (not MILP)
# m.addConstr(x2 == y*x1, name="x2_y_x1")

####################################
# create the constraint the MILP way
x_1p = m.addVar(lb=0, ub=gp.GRB.INFINITY, name="x_1p")
x_1m = m.addVar(lb=0, ub=gp.GRB.INFINITY, name="x_1m")
s_p = m.addVar(lb=0, ub=gp.GRB.INFINITY, name="s_p")
s_m = m.addVar(lb=0, ub=gp.GRB.INFINITY, name="s_m")
b = m.addVar(vtype=gp.GRB.BINARY, name="b")

m.addConstr(x1 == x_1p - x_1m, name="x1_split")
m.addConstr(x_1p <= x1_ub*b, name="x1p_ub")
m.addConstr(x_1m <= (1-b)*x1_ub, name="x1p_ub2")

# now we can write two constraints for positive x_1p and x_1m
m.addConstr(s_p >= y_lb*x_1p, name="s_p_lb")
m.addConstr(s_p <= y_ub*x_1p, name="s_p_ub")
m.addConstr(s_p >= y*x1_lb, name="s_p_lb2")
m.addConstr(s_p <= y*x1_ub, name="s_p_ub2") 

m.addConstr(s_m >= y_lb*x_1m, name="s_m_lb")
m.addConstr(s_m <= y_ub*x_1m, name="s_m_ub")
m.addConstr(s_m >= y*x1_lb, name="s_m_lb2")
m.addConstr(s_m <= y*x1_ub, name="s_m_ub2")

m.addConstr(x2 == s_p - s_m, name="x2_split")

# optimize
m.setObjective(x2, gp.GRB.MAXIMIZE)

t0 = time.time()
m.optimize()
print(f"Optimization took {time.time()-t0} seconds")
print(f"Code: {scs[m.status]}")

if m.status == gp.GRB.INFEASIBLE:
        m.computeIIS()
        print("IIS:")
        for c in m.getConstrs():
            if c.IISConstr: print(f'\t{c.constrname}: {m.getRow(c)} {c.Sense} {c.RHS}')

# Print the results
print(f"obj: {m.objVal}")
print(f"x1: {x1.X}")
print(f"y: {y.X}")
print(f"x1_p: {x_1p.X}")
print(f"x1_m: {x_1m.X}")
print(f"s_p: {s_p.X}")
print(f"s_m: {s_m.X}")
