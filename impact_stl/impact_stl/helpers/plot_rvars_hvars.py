import matplotlib.pyplot as plt
import os

from planner.utilities.beziers import eval_bezier, get_derivative_control_points_gurobi

def plot_rvars_hvars(rvars,hvars,path="",fn="plot_rvars_hvars.png"):
    # input is just an array of rvars and an 
    # array of hvars (of assumed identical length).
    nbzs = len(rvars)

    # convert rvars and hvars to also have drvars and dhvars
    drvars = [get_derivative_control_points_gurobi(rvar) for rvar in rvars]
    dhvars = [get_derivative_control_points_gurobi(hvar) for hvar in hvars]
    ddrvars = [get_derivative_control_points_gurobi(drvar) for drvar in drvars]
    ddhvars = [get_derivative_control_points_gurobi(dhvar) for dhvar in dhvars]

    # evaluate all the bezier
    revals = [eval_bezier(rvar,N=20) for rvar in rvars]
    drevals = [eval_bezier(drvar,N=20) for drvar in drvars]
    ddrevals = [eval_bezier(ddrvar,N=20) for ddrvar in ddrvars]
    hevals = [eval_bezier(hvar,N=20) for hvar in hvars]
    dhevals = [eval_bezier(dhvar,N=20) for dhvar in dhvars]
    ddhevals = [eval_bezier(ddhvar,N=20) for ddhvar in ddhvars]

    dqevals = [dreval/dheval for dreval,dheval in zip(drevals,dhevals)]
    ddqevals = [(ddreval*dheval - dreval*ddheval)/(dheval**2) 
                for dreval,dheval,ddreval,ddheval in zip(drevals,dhevals,ddrevals,ddhevals)]
    
    # and plot it
    fig, axs = plt.subplots(3,3,figsize=(15,15))

    for bz in range(nbzs):
        axs[0,0].plot(revals[bz][0,:],revals[bz][1,:])
        axs[0,0].plot(revals[bz][0,-1],revals[bz][1,-1],'ro')
    axs[0,0].set_title("x-y plane")
    axs[0,0].set_xlabel("x [m]")
    axs[0,0].set_ylabel("y [m]")
    axs[0,0].grid(True)
    axs[0,0].set_aspect('equal')
    

    for bz in range(nbzs):
        axs[0,1].plot(hevals[bz][0,:],revals[bz][0,:])
        axs[0,1].plot(hevals[bz][0,-1],revals[bz][0,-1],'ro')
    axs[0,1].set_title("time-x")
    axs[0,1].set_xlabel("t [s]")
    axs[0,1].set_ylabel("x [m]")
    axs[0,1].grid(True)

    for bz in range(nbzs):
        axs[0,2].plot(hevals[bz][0,:],revals[bz][1,:])
        axs[0,2].plot(hevals[bz][0,-1],revals[bz][1,-1],'ro')
    axs[0,2].set_title("time-y")
    axs[0,2].set_xlabel("t [s]")
    axs[0,2].set_ylabel("y [m]")
    axs[0,2].grid(True)

    # row 2
    for bz in range(nbzs):
        axs[1,0].plot(hevals[bz][0,:])
    axs[1,0].set_title('h-t plane')
    axs[1,0].set_xlabel("t [s]")
    axs[1,0].set_ylabel("s [-]")
    axs[1,0].grid(True)

    ## velocity
    for bz in range(nbzs):
        axs[1,1].plot(hevals[bz][0,:],dqevals[bz][0,:])
        axs[1,1].plot(hevals[bz][0,-1],dqevals[bz][0,-1],'ro')
    axs[1,1].set_title("time-dx")
    axs[1,1].set_xlabel("t [s]")
    axs[1,1].set_ylabel("dx [m/s]")
    axs[1,1].grid(True)

    for bz in range(nbzs):
        axs[1,2].plot(hevals[bz][0,:],dqevals[bz][1,:])
        axs[1,2].plot(hevals[bz][0,-1],dqevals[bz][1,-1],'ro')
    axs[1,2].set_title("time-dy")
    axs[1,2].set_xlabel("t [s]")
    axs[1,2].set_ylabel("dy [m/s]")
    axs[1,2].grid(True)

    ## acceleration
    for bz in range(nbzs):
        axs[2,1].plot(hevals[bz][0,:],ddqevals[bz][0,:])
        axs[2,1].plot(hevals[bz][0,-1],ddqevals[bz][0,-1],'ro')
    axs[2,1].set_title("time-ddx")
    axs[2,1].set_xlabel("t [s]")
    axs[2,1].set_ylabel("ddx [m/s^2]")
    axs[2,1].grid(True)

    for bz in range(nbzs):
        axs[2,2].plot(hevals[bz][0,:],ddqevals[bz][1,:])
        axs[2,2].plot(hevals[bz][0,-1],ddqevals[bz][1,-1],'ro')
    axs[2,2].set_title("time-ddy")
    axs[2,2].set_xlabel("t [s]")
    axs[2,2].set_ylabel("ddy [m/s^2]")
    axs[2,2].grid(True)


    plt.savefig(os.path.join(path,fn), dpi=150)
