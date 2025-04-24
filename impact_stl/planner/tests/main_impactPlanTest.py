# define a qp solver for just the pre-impact and post-impact Bezier planner
# we know a post-impact velocity of the object from the planner as well as 
# the impact time, we can compute the pre-impact velocity of the robot based 
# on the observed pre-impact velocity and position of the object

import numpy as np
import cvxpy as cp
import sys
sys.path.append('..')
import matplotlib.pyplot as plt
# from push_stl.planner.utilities.beziers import eval_bezier
from utilities.beziers import eval_bezier
import time

x0 = np.array([-1,-1])
dx0 = np.array([0.1,0.1])

xf = np.array([-1,1])
dxf = np.array([-0.1,0.1])

tf = 2.0

# object properties
mr = 15
mo = 15
radius = 0.15

m1 = (mr-mo)/(mr+mo)
m2 = 2*mo/(mr+mo)
m3 = 2*mr/(mr+mo)
m4 = (mo-mr)/(mr+mo)


t = 0.0
t_impact = 1.0

xobject = np.array([1,0])
dxobject = np.array([-0.5,0.0])

x_at_impact = xobject + dxobject*(t_impact-t)
dxobject_desired = np.array([0.0,0.5])

dx_vector = (-dxobject_desired + dxobject)/np.linalg.norm(-dxobject_desired + dxobject)
# compute the position on the boundary of the robot
x_boundary = x_at_impact + radius*dx_vector
x_center   = x_at_impact + 2*radius*dx_vector


# optimization variables
t0 = time.time()

order = 4
rvar = [cp.Variable((2,order+1)) for i in range(2)]
drvar = [cp.Variable((2,order)) for i in range(2)]
ddrvar = [cp.Variable((2,order-1)) for i in range(2)]

hvar = [cp.Variable((1,5)) for i in range(2)]
dhvar = [cp.Variable((1,4)) for i in range(2)]
ddhvar = [cp.Variable((1,3)) for i in range(2)]

# constraints
constraints = []
# bezier derivative constraints
constraints += [drvar[i][:,j] == order*(rvar[i][:,j+1] - rvar[i][:,j]) for i in range(2) for j in range(order)]
constraints += [ddrvar[i][:,j] == order*(drvar[i][:,j+1] - drvar[i][:,j]) for i in range(2) for j in range(order-1)]
constraints += [dhvar[i][0,j] == order*(hvar[i][0,j+1] - hvar[i][0,j]) for i in range(2) for j in range(order)]
constraints += [ddhvar[i][0,j] == order*(dhvar[i][0,j+1] - dhvar[i][0,j]) for i in range(2) for j in range(order-1)]
# initial and final states
constraints += [rvar[0][:,0] == x0]
constraints += [rvar[-1][:,-1] == xf]
constraints += [drvar[0][:,0] == dx0]
constraints += [drvar[-1][:,-1] == dxf]
constraints += [hvar[0][:,0] == 0]
constraints += [hvar[-1][:,-1] == tf]
# continuity constraints
constraints += [hvar[0][:,-1] == hvar[-1][:,0]]
constraints += [rvar[0][:,-1] == rvar[-1][:,0]]
# time derivative consraints
constraints += [dhvar[0][0,i] >= 0.01 for i in range(4)]
constraints += [dhvar[-1][0,i] >= 0.01 for i in range(4)]
# time derivative impact constraints
dh_robot = 1
constraints += [dhvar[0][0,-1] == dh_robot]
constraints += [dhvar[-1][0,0] == dh_robot]

# impact constraints
constraints += [hvar[0][:,-1] == t_impact]
constraints += [rvar[0][:,-1] == x_center]
constraints += [drvar[-1][:,0] == m1*drvar[0][:,-1] + m2*dxobject]
constraints += [dxobject_desired == m3*drvar[0][:,-1] + m4*dxobject]

# penalize ddrvar'T @ ddrvar + ddhvar'T @ ddhvar
cost = 0
for i in range(2):
    cost += cp.sum_squares(ddrvar[i]) + cp.sum_squares(ddhvar[i])

# solve the optimization problem
prob = cp.Problem(cp.Minimize(cost),constraints)
prob.solve()

rvals = [eval_bezier(rvar[i].value) for i in range(2)]
drvals = [eval_bezier(drvar[i].value) for i in range(2)]
ddrvals = [eval_bezier(ddrvar[i].value) for i in range(2)]
hvals = [eval_bezier(hvar[i].value) for i in range(2)]
dhvals = [eval_bezier(dhvar[i].value) for i in range(2)]
ddhvals = [eval_bezier(ddhvar[i].value) for i in range(2)]

qvals = rvals
dqvals = [drvals[i]/dhvals[i] for i in range(2)]
ddqvals = [(ddrvals[i]-dqvals[i]*ddhvals[i])/(dhvals[i]**2) for i in range(2)]


print(f"Optimization took {time.time()-t0} seconds")


fig, axs = plt.subplots(3,3)
axs = axs.flatten()

axs[0].plot(qvals[0][0,:],qvals[0][1,:],'b')
axs[0].plot(qvals[1][0,:],qvals[1][1,:],'b')
# draw a circle of radius around qvals[0][:,-1]
circle = plt.Circle(qvals[0][:,-1],radius,fill=False)
axs[0].add_artist(circle)
axs[0].plot(xobject[0],xobject[1],'ro')
axs[0].plot(x_at_impact[0],x_at_impact[1],'go')
# draw a circle of radius around x_at_impact
circle = plt.Circle(x_at_impact,radius,fill=False)
axs[0].add_artist(circle)
# plot an arrow of the dxobject at impact
axs[0].arrow(x_at_impact[0],x_at_impact[1],0.3*dxobject[0],0.3*dxobject[1],head_width=0.05,head_length=0.05)
# and the desired velocity at impact
axs[0].arrow(x_at_impact[0],x_at_impact[1],0.3*dxobject_desired[0],0.3*dxobject_desired[1],head_width=0.05,head_length=0.05)
# axis equal
# grid on
axs[0].grid()
axs[0].set_aspect('equal')

axs[1].plot(hvals[0][0,:],qvals[0][0,:],'b')
axs[1].plot(hvals[1][0,:],qvals[1][0,:],'b')
axs[1].plot([t_impact,t_impact],[0,1],'r')
axs[1].set_xlabel('time')
axs[1].set_ylabel('x')
axs[1].grid()

axs[2].plot(hvals[0][0,:],qvals[0][1,:],'b')
axs[2].plot(hvals[1][0,:],qvals[1][1,:],'b')
axs[2].plot([t_impact,t_impact],[0,1],'r')
axs[2].set_xlabel('time')
axs[2].set_ylabel('y')
axs[2].grid()

axs[3].plot(hvals[0][0,:],dhvals[0][0,:],'b')
axs[3].plot(hvals[1][0,:],dhvals[1][0,:],'b')
axs[3].set_xlabel('time')
axs[3].set_ylabel('dh')
axs[3].grid()

axs[4].plot(hvals[0][0,:],dqvals[0][0,:],'b')
axs[4].plot(hvals[1][0,:],dqvals[1][0,:],'b')
axs[4].set_xlabel('time')
axs[4].set_ylabel('dx')
axs[4].grid()

axs[5].plot(hvals[0][0,:],dqvals[0][1,:],'b')
axs[5].plot(hvals[1][0,:],dqvals[1][1,:],'b')
axs[5].set_xlabel('time')
axs[5].set_ylabel('dy')
axs[5].grid()

axs[6].plot(hvals[0][0,:],ddhvals[0][0,:],'b')
axs[6].plot(hvals[1][0,:],ddhvals[1][0,:],'b')
axs[6].set_xlabel('time')
axs[6].set_ylabel('ddh')
axs[6].grid()

axs[7].plot(hvals[0][0,:],ddqvals[0][0,:],'b')
axs[7].plot(hvals[1][0,:],ddqvals[1][0,:],'b')
axs[7].set_xlabel('time')
axs[7].set_ylabel('ddx')
axs[7].grid()

axs[8].plot(hvals[0][0,:],ddqvals[0][1,:],'b')
axs[8].plot(hvals[1][0,:],ddqvals[1][1,:],'b')
axs[8].set_xlabel('time')
axs[8].set_ylabel('ddy')
axs[8].grid()

plt.show()


