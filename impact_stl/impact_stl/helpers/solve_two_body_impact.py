#!/usr/bin/env python
import casadi as cs
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

#TODO: add angular momentum considerations
def Rot(x):
    return np.array([[np.cos(x),np.sin(x)],
                     [-np.sin(x),np.cos(x)]])

def invRot(x):
    return np.array([[np.cos(x),-np.sin(x)],
                     [np.sin(x),np.cos(x)]])

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

def solve_two_body_impact(xObjectI,dxObjectI,dxObjectI_post,
                          dx_init_guess=None,
                          theta_init_guess=None,
                          verbose=False):
    print("\n\nStarting two-body-impact construction") if verbose else None
    # here a slightly smaller radius works better so that the impact is guaranteed
    # as we adapt the weights of the MPC, this is warranted.
    r_R = 0.20 # was 0.15
    r_O = 0.20 # was 0.15
    m_R = 16.8
    m_O = 16.8
    e = 0.1 # c for sim
    # e = 0.4615 # c for hw, tire around one robot
    # e = (0.37+0.30+0.43+0.45)/4 # c for hw, tire around two robots
    # e = 0.25 # just testing on Thursday

    print(f"xObjectI: {xObjectI}") if verbose else None
    print(f"dxObjectI: {dxObjectI}") if verbose else None
    print(f"dxObjectI_post: {dxObjectI_post}") if verbose else None

    ocp = cs.Opti()

    thetavar = ocp.variable(1)
    x_R_G, x_O_G = ocp.variable(2,1), np.array([0.,0.])
    dx_R_G_pre, dx_O_G_pre = ocp.variable(2,1), dxObjectI
    dx_R_G_post, dx_O_G_post = ocp.variable(2,1), ocp.variable(2,1)# dxObjectI_post
    dx_R_L_pre, dx_O_L_pre = ocp.variable(2,1), ocp.variable(2,1)
    dx_R_L_post, dx_O_L_post = ocp.variable(2,1), ocp.variable(2,1)

    # set initial conditions 
    if theta_init_guess is not None:
        print(f"theta_init_guess: {theta_init_guess} ({np.rad2deg(theta_init_guess)} deg)") if verbose else None
        ocp.set_initial(thetavar,theta_init_guess)
        ocp.set_initial(x_R_G[0],np.cos(theta_init_guess)*(r_R+r_O))
        ocp.set_initial(x_R_G[1],np.sin(theta_init_guess)*(r_R+r_O))
    if dx_init_guess is not None:
        print(f"dx_init_guess: {dx_init_guess}") if verbose else None
        ocp.set_initial(dx_R_G_pre,dx_init_guess)
    if theta_init_guess is not None and dx_init_guess is not None:
        ocp.set_initial(dx_R_L_pre,Rot(theta_init_guess)@dx_init_guess)
        # ocp.set_initial(dx_R_G_post,invRot(theta_init_guess)@Rot(theta_init_guess)@dx_init_guess)

    # impacts on the boundary of the robor
    ocp.subject_to(x_R_G[0] == cs.cos(thetavar)*(r_R+r_O))
    ocp.subject_to(x_R_G[1] == cs.sin(thetavar)*(r_R+r_O))
    # set pre-impact velocity of object (now done via setup)
    # set post-impact velocity of object (desired) (now done via setup)
    # set constraints to obtain velocities in local frame
    ocp.subject_to(dx_R_L_pre == Rot_cs(thetavar)@dx_R_G_pre)
    ocp.subject_to(dx_O_L_pre == Rot_cs(thetavar)@dx_O_G_pre)
    # set constraints to obtain post-impact velocities in global frame
    ocp.subject_to(dx_R_G_post == invRot_cs(thetavar)@dx_R_L_post)
    ocp.subject_to(dx_O_G_post == invRot_cs(thetavar)@dx_O_L_post)
    # set linear set of equations constraints
    A = np.array([[m_R,0,m_O,0],
                  [1,0,-1,0],
                  [0,m_R,0,0],
                  [0,0,0,m_O]])
    b = ocp.variable(4)
    ocp.subject_to(b[0] == m_R*dx_R_L_pre[0] + m_O*dx_O_L_pre[0])
    ocp.subject_to(b[1] == -e*(dx_R_L_pre[0] - dx_O_L_pre[0]))
    ocp.subject_to(b[2] == m_R*dx_R_L_pre[1])
    ocp.subject_to(b[3] == m_O*dx_O_L_pre[1])
    x = cs.vertcat(dx_R_L_post,dx_O_L_post)
    ocp.subject_to(A@x == b)
    # constrain that pos and vel vector point towards eachother 
    x_G_R_in_O = ocp.variable(2,1)
    dx_G_R_in_O = ocp.variable(2,1)
    ocp.subject_to(x_G_R_in_O == x_R_G - x_O_G)
    ocp.subject_to(dx_G_R_in_O == dx_R_G_pre - dx_O_G_pre)
    ocp.subject_to(x_G_R_in_O[0]*dx_G_R_in_O[0] + x_G_R_in_O[1]*dx_G_R_in_O[1] <= 0)

    # set cost: rms w.r.t. desired
    ocp.minimize((dx_O_G_post - dxObjectI_post)[0]**2 + (dx_O_G_post - dxObjectI_post)[1]**2 + \
                 0.0001*(dx_init_guess[0] - dx_R_G_pre[0])**2 + 0.0001*(dx_init_guess[1] - dx_R_G_pre[1])**2)
    # # or a feasibility problem
    ocp.set_initial(dx_O_G_post,dxObjectI_post)
    # ocp.subject_to(dx_O_G_post == dxObjectI_post)

    # set solver method
    opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
    ocp.solver('ipopt',opts)

    # solve
    t0 = time.time()
    sol = ocp.solve()
    print(f"\n\nSolving 2bp took {time.time()-t0} seconds") if verbose else None

    # parse solution
    thetaval = sol.value(thetavar)
    print(f"theta: {thetaval} ({np.rad2deg(thetaval)} deg)") if verbose else None

    theta_val = sol.value(thetavar)
    x_R_G_val, x_O_G_val = sol.value(x_R_G), np.array([0.,0.])
    dx_R_G_pre_val, dx_O_G_pre_val = sol.value(dx_R_G_pre), dxObjectI
    dx_R_G_post_val, dx_O_G_post_val = sol.value(dx_R_G_post), sol.value(dx_O_G_post)# dxObjectI_post
    dx_R_L_pre_val, dx_O_L_pre_val = sol.value(dx_R_L_pre), sol.value(dx_O_L_pre)
    dx_R_L_post_val, dx_O_L_post_val = sol.value(dx_R_L_post), sol.value(dx_O_L_post)

    if verbose:
        print(f"dx_R_L_pre: {dx_R_L_pre_val}")
        print(f"dx_O_L_pre: {dx_O_L_pre_val}")
        print(f"dx_R_G_pre: {dx_R_G_pre_val}")
        print(f"dx_O_G_pre: {dx_O_G_pre_val}")

        print(f"\ndx_R_L_post: {dx_R_L_post_val}")
        print(f"dx_O_L_post: {dx_O_L_post_val}")
        print(f"dx_R_G_post: {dx_R_G_post_val}")
        print(f"dx_O_G_post: {dx_O_G_post_val} (desired: {dxObjectI_post})")

    # plotting
    if verbose:
        fig, axs = plt.subplots(1,2)
        axs[0].plot(x_R_G_val[0],x_R_G_val[1],'ro')
        axs[0].add_patch(patches.Circle(x_R_G_val,r_R,edgecolor='red',facecolor='none'))
        axs[0].quiver(x_R_G_val[0],x_R_G_val[1],dx_R_G_pre_val[0],dx_R_G_pre_val[1],scale_units='xy',scale=2,color='red')

        axs[0].plot(x_O_G_val[0],x_O_G_val[1],'bo')
        axs[0].add_patch(patches.Circle(x_O_G_val,r_O,edgecolor='blue',facecolor='none'))
        axs[0].quiver(x_O_G_val[0],x_O_G_val[1],dx_O_G_pre_val[0],dx_O_G_pre_val[1],scale_units='xy',scale=2,color='blue')

        axs[0].set_aspect('equal')
        axs[0].grid(True)
        axs[0].set_title('pre-impact')

        axs[1].plot(x_R_G_val[0],x_R_G_val[1],'ro')
        axs[1].add_patch(patches.Circle(x_R_G_val,r_R,edgecolor='red',facecolor='none'))
        axs[1].quiver(x_R_G_val[0],x_R_G_val[1],dx_R_G_post_val[0],dx_R_G_post_val[1],scale_units='xy',scale=2,color='red')

        axs[1].plot(x_O_G_val[0],x_O_G_val[1],'bo')
        axs[1].add_patch(patches.Circle(x_O_G_val,r_O,edgecolor='blue',facecolor='none'))
        axs[1].quiver(x_O_G_val[0],x_O_G_val[1],dx_O_G_post_val[0],dx_O_G_post_val[1],scale_units='xy',scale=2,color='blue')

        axs[1].set_aspect('equal')
        axs[1].grid(True)
        axs[1].set_title('post-impact')
        plt.savefig("/home/px4space/space_ws/p_two_body_impact.png")

    xI = x_R_G_val + xObjectI
    dxI = dx_R_G_pre_val
    xI_post = x_R_G_val + xObjectI
    dxI_post = dx_R_G_post_val
    return xI, dxI, xI_post, dxI_post


# ##########
# ## TEST ##
# ##########
# xObjectI = np.array([3.,0.])
# dxObjectI = np.array([-1,0.])
# dxObjectI_post = np.array([0.1,0.0])

# theta_init_guess = np.pi
# dx_init_guess = np.array([0.05,0.0])

# solve_two_body_impact(xObjectI,dxObjectI,dxObjectI_post,
#                       theta_init_guess=theta_init_guess,
#                       dx_init_guess=dx_init_guess,
#                       verbose=True)

# solve_two_body_impact_jit(xObjectI,dxObjectI,dxObjectI_post,
#                           theta_init_guess=theta_init_guess,
#                           dx_init_guess=dx_init_guess,
#                           verbose=True)

