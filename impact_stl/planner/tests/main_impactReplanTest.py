import casadi as cs
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import scipy as sp
import time

m_A = 2
m_B = 11

r_A = 1.0
r_B = 2.0

e = 0.6

Rot = lambda x: np.array([[np.cos(x), np.sin(x)],
                          [-np.sin(x), np.cos(x)]])

invRot = lambda x: np.array([[np.cos(x), -np.sin(x)],
                             [np.sin(x), np.cos(x)]])

# First, we test the impact equations for two non-rotating circles 
# with given positions and velocities
# https://www.kpu.ca/sites/default/files/Faculty%20of%20Science%20%26%20Horticulture/Physics/Ch15%20-%203%20-%20Impact_0.pdf#:~:text=Impact%20occurs%20when%20two%20bodies%20collide%20during%20a,through%20the%20mass%20centers%20of%20the%20colliding%20particles.
theta = np.deg2rad(-70.529)
x_B_G = np.array([0,0])
x_A_G = np.array([np.cos(theta)*(r_A+r_B), np.sin(theta)*(r_A+r_B)])

# velocities in the global frame
dx_B_G = np.array([0,0])
dx_A_G = np.array([0,3])

# velocities in the local frame
dx_A_L = Rot(theta)@dx_A_G 
dx_B_L = Rot(theta)@dx_B_G
print(f"dx_A_L: {dx_A_L}")
print(f"dx_B_L: {dx_B_L}")

# now we get 4 eqs with 4 unknowns in the local _L frame!
A = np.zeros((4,4))
b = np.zeros((4,))
x = ['v^{+}_{Ax}','v^{+}_{Bx}','v^{+}_{Ay}','v^{+}_{By}']
# 1: conservation of momentum around x
A[0,:] = np.array([m_A,m_B,0,0])
b[0] = m_A*dx_A_L[0] + m_B*dx_B_L[0]
# 2: conservation of momentum around y
A[1,:] = np.array([0,0,m_A,0])
b[1] = m_A*dx_A_L[1]
# 3: energy dissapation around x
A[2,:] = np.array([1,-1,0,0])
b[2] = -e*(dx_A_L[0] - dx_B_L[0])
# 4: energy dissapation around y
A[3,:] = np.array([0,0,0,m_B])
b[3] = m_B*dx_B_L[1]

# post-impact velocities in the local frame
print(f"\nA: {A}")
print(f"b: {b}")
x = np.linalg.solve(A,b)
print(f"\nx: {x}")
print(f"Ax: {A@x}")

dx_A_L_plus = x[[0,2]]
dx_B_L_plus = x[[1,3]]
print(f"\ndx_A_L_plus: {dx_A_L_plus}")
print(f"dx_B_L_plus: {dx_B_L_plus}")
# and convert back to global frame velocities
dx_A_G_plus = invRot(theta)@dx_A_L_plus
dx_B_G_plus = invRot(theta)@dx_B_L_plus
print(f"dx_A_G_plus: {dx_A_G_plus}")
print(f"dx_B_G_plus: {dx_B_G_plus}")

# plotting
fig, axs = plt.subplots(1,2)
axs[0].plot(x_A_G[0],x_A_G[1],'ro')
axs[0].add_patch(patches.Circle(x_A_G,r_A,edgecolor='red',facecolor='none'))
axs[0].quiver(x_A_G[0],x_A_G[1],dx_A_G[0],dx_A_G[1],scale_units='xy',scale=2,color='red')

axs[0].plot(x_B_G[0],x_B_G[1],'bo')
axs[0].add_patch(patches.Circle(x_B_G,r_B,edgecolor='blue',facecolor='none'))
axs[0].quiver(x_B_G[0],x_B_G[1],dx_B_G[0],dx_B_G[1],scale_units='xy',scale=2,color='blue')

axs[0].set_aspect('equal')
axs[0].grid(True)
axs[0].set_title('pre-impact')

axs[1].plot(x_A_G[0],x_A_G[1],'ro')
axs[1].add_patch(patches.Circle(x_A_G,r_A,edgecolor='red',facecolor='none'))
axs[1].quiver(x_A_G[0],x_A_G[1],dx_A_G_plus[0],dx_A_G_plus[1],scale_units='xy',scale=2,color='red')

axs[1].plot(x_B_G[0],x_B_G[1],'bo')
axs[1].add_patch(patches.Circle(x_B_G,r_B,edgecolor='blue',facecolor='none'))
axs[1].quiver(x_B_G[0],x_B_G[1],dx_B_G_plus[0],dx_B_G_plus[1],scale_units='xy',scale=2,color='blue')

axs[1].set_aspect('equal')
axs[1].grid(True)
axs[1].set_title('post-impact')
plt.savefig("figures/linear_impact_circles.png")





#TODO: add angular momentum considerations
#TODO: create optimization problem with the constraint:
#       s.t. dot(r_A_to_B,dr_A_to_B) \geq 0   at point of impact

def Rot_cs(x):
    R = cs.MX(2,2)
    R[0,0], R[0,1] = cs.cos(x), cs.sin(x)
    R[1,0], R[1,1] = -cs.sin(x), cs.cos(x)
    return R

def invRot_cs(x):
    R = cs.MX(2,2)
    R[0,0], R[0,1] = cs.cos(x), -cs.sin(x)
    R[1,0], R[1,1] = cs.sin(x), cs.cos(x)
    return R

ocp = cs.Opti()

thetavar = ocp.variable(1)
dx_A_B_G_pre = ocp.variable(4,1)
dx_A_B_G_post = ocp.variable(4,1)
dx_A_B_L_pre = ocp.variable(4,1)
dx_A_B_L_post = ocp.variable(4,1)

x_A_B_G = ocp.variable(4,1)
ocp.subject_to(x_A_B_G[0] == cs.cos(thetavar)*(r_A+r_B))
ocp.subject_to(x_A_B_G[1] == cs.sin(thetavar)*(r_A+r_B))
ocp.subject_to(x_A_B_G[2] == 0.0)
ocp.subject_to(x_A_B_G[3] == 0.0)

# set pre-impact velocity of object
ocp.subject_to(dx_A_B_G_pre[2:4] == np.array([0.,0.]))
# set post-impact velocity of object (desired)
ocp.subject_to(dx_A_B_G_post[2:4] == np.array([-0.1,0.65]))
# set constraints to obtain velocities in local frame
ocp.subject_to(dx_A_B_L_pre[0:2] == Rot_cs(thetavar)@dx_A_B_G_pre[0:2])
ocp.subject_to(dx_A_B_L_pre[2:4] == Rot_cs(thetavar)@dx_A_B_G_pre[2:4])
# set constraints to obtain post-impact velocities in global frame
ocp.subject_to(dx_A_B_G_post[0:2] == invRot_cs(thetavar)@dx_A_B_L_post[0:2])
ocp.subject_to(dx_A_B_G_post[2:4] == invRot_cs(thetavar)@dx_A_B_L_post[2:4])
# set linear set of equations constraints
A = np.array([[m_A,0,m_B,0],
              [1,0,-1,0],
              [0,m_A,0,0],
              [0,0,0,m_B]])
b = ocp.variable(4)
ocp.subject_to(b[0] == m_A*dx_A_B_L_pre[0] + m_B*dx_A_B_L_pre[2])
ocp.subject_to(b[1] == -e*(dx_A_B_L_pre[0] - dx_A_B_L_pre[2]))
ocp.subject_to(b[2] == m_A*dx_A_B_L_pre[1])
ocp.subject_to(b[3] == m_B*dx_A_B_L_pre[3])
ocp.subject_to(A@dx_A_B_L_post == b)
# constrain that pos and vel vector point towards eachother 
x_G_A_in_B = ocp.variable(2,1)
dx_G_A_in_B = ocp.variable(2,1)
ocp.subject_to(x_G_A_in_B == x_A_B_G[0:2] - x_A_B_G[2:4])
ocp.subject_to(dx_G_A_in_B == dx_A_B_G_pre[0:2] - dx_A_B_G_pre[0:2])
ocp.subject_to(x_G_A_in_B[0]*dx_G_A_in_B[0] + x_G_A_in_B[1]*dx_G_A_in_B[1] <= 0)


# set solver method
opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
ocp.solver('ipopt',opts)
# solve
t0 = time.time()
sol = ocp.solve()
print(f"\n\nSolving took {time.time()-t0} seconds")

# parse solution
thetaval = sol.value(thetavar)
print(f"theta: {thetaval} ({np.rad2deg(thetaval)} deg)")

dx_A_B_G_pre_val = sol.value(dx_A_B_G_pre)
dx_A_B_G_post_val = sol.value(dx_A_B_G_post)
dx_A_B_L_pre_val = sol.value(dx_A_B_L_pre)
dx_A_B_L_post_val = sol.value(dx_A_B_L_post)

print(f"dx_A_L_plus: {dx_A_B_L_post_val[0:2]}")
print(f"dx_B_L_plus: {dx_A_B_L_post_val[2:4]}")
print(f"dx_A_G_plus: {dx_A_B_G_post_val[0:2]}")
print(f"dx_B_G_plus: {dx_A_B_G_post_val[2:4]}")

x_A_G = sol.value(x_A_B_G)[0:2]
x_B_G = sol.value(x_A_B_G)[2:4]
dx_A_G = dx_A_B_G_pre_val[0:2]
dx_B_G = dx_A_B_G_pre_val[2:4]
dx_A_G_plus = dx_A_B_G_post_val[0:2]
dx_B_G_plus = dx_A_B_G_post_val[2:4]


# plotting
fig, axs = plt.subplots(1,2)
axs[0].plot(x_A_G[0],x_A_G[1],'ro')
axs[0].add_patch(patches.Circle(x_A_G,r_A,edgecolor='red',facecolor='none'))
axs[0].quiver(x_A_G[0],x_A_G[1],dx_A_G[0],dx_A_G[1],scale_units='xy',scale=2,color='red')

axs[0].plot(x_B_G[0],x_B_G[1],'bo')
axs[0].add_patch(patches.Circle(x_B_G,r_B,edgecolor='blue',facecolor='none'))
axs[0].quiver(x_B_G[0],x_B_G[1],dx_B_G[0],dx_B_G[1],scale_units='xy',scale=2,color='blue')

axs[0].set_aspect('equal')
axs[0].grid(True)
axs[0].set_title('pre-impact')

axs[1].plot(x_A_G[0],x_A_G[1],'ro')
axs[1].add_patch(patches.Circle(x_A_G,r_A,edgecolor='red',facecolor='none'))
axs[1].quiver(x_A_G[0],x_A_G[1],dx_A_G_plus[0],dx_A_G_plus[1],scale_units='xy',scale=2,color='red')

axs[1].plot(x_B_G[0],x_B_G[1],'bo')
axs[1].add_patch(patches.Circle(x_B_G,r_B,edgecolor='blue',facecolor='none'))
axs[1].quiver(x_B_G[0],x_B_G[1],dx_B_G_plus[0],dx_B_G_plus[1],scale_units='xy',scale=2,color='blue')

axs[1].set_aspect('equal')
axs[1].grid(True)
axs[1].set_title('post-impact')
plt.savefig("figures/opt_linear_impact_circles.png")
