import gurobipy as gp
# scs = {v: k for k, v in vars(gp.StatusConstClass).items() if k[0].isupper()}
scs = {getattr(gp.GRB.status,k): k for k in dir(gp.GRB.status) if k[0].isupper()}

import time
import numpy as np
import scipy as sp
from scipy import optimize
import pickle

# plotting imports
import scienceplots
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import rc
from matplotlib.axes import Axes
from matplotlib.patches import Patch
from matplotlib.gridspec import GridSpec
from matplotlib import animation
# Set up plot style
plt.style.use(['science'])
rc('text', usetex=True)
rc('font', family='times', size=35)

plt.rcParams['legend.frameon'] = True             # Enable legend frame
plt.rcParams['legend.facecolor'] = 'white'        # Set background color
plt.rcParams['legend.edgecolor'] = 'white'        # Set border color
plt.rcParams['legend.framealpha'] = 1.0
plt.rcParams['legend.loc'] = 'best'

# custom imports
from utilities.sr_stl import quant_parse_operator, parse_time_robot_robot
from utilities.beziers import get_derivative_control_points_gurobi, eval_bezier, value_bezier
from World import World

class SR_Impact_STL:
    def __init__(self,world: World):
        # load the STL specification, with x0 and xf for all robots and objects
        # self.spec = world.spec
        self.world = world
        self.robots = world.robots
        self.objects = world.objects

        # restitution coefficient
        self.e = 0.47 # we used 0.47 for the experiments pingpong

        self.bigM = 1e4
        self.dh_lb = 1e-1
        self.dh_ub = 1e3

    def construct(self):
        self.prog = gp.Model("impact_stl")
        self.prog.setParam(gp.GRB.Param.OutputFlag, 1)

        ######################
        # set robot parameters
        self.nrobots = len(self.robots)
        self.robots_nbzs = np.array([robot.nbz for robot in self.robots])
        self.robots_order = np.array([5 for robot in self.robots])
        self.robots_ncp   = self.robots_order + 1

        #######################
        # set object parameters
        self.nobjects = len(self.objects)
        self.objects_nbzs = np.array([obj.nbz for obj in self.objects])     # number bzs for each object, should be 4*k + 1
        self.objects_order = np.array([1 for obj in self.objects])          # order of 'wait', 'push', 'float', 'brake'
        self.objects_ncp   = self.objects_order + 1
        # # now create objects_order which, for each robots, fills the nbz with the object_order
        # self.objects_order = [np.zeros(self.objects_nbzs[o],dtype=int) for o in range(len(self.objects))]
        # # now fil the 4*k+1 elements with the object_order
        # for o in range(len(self.objects)):
        #     for i in range(4):
        #         self.objects_order[o][i::4] = int(self.object_order[i])
        #     self.objects_order[o][-1] = 1
        # self.objects_ncp   = [self.objects_order[o]+1 for o in range(self.nobjects)]

        ##########################################
        # add continuous state and input variables
        self.robots_rvar = [[self.prog.addMVar((self.world.dim,self.robots_ncp[r]),
                             lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_hvar = [[self.prog.addMVar((1,self.robots_ncp[r]),
                             lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        
        self.objects_rvar = [[self.prog.addMVar((self.world.dim,self.objects_ncp[o]),
                              lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY) for i in range(self.objects_nbzs[o])] for o in range(len(self.objects))]
        self.objects_hvar = [[self.prog.addMVar((1,self.objects_ncp[o]),
                              lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY) for i in range(self.objects_nbzs[o])] for o in range(len(self.objects))]
        self.prog.update()

        # Now obtain the control points of the derivatives as linear combinations of the control points of the original bzs
        self.robots_drvar = [[get_derivative_control_points_gurobi(self.robots_rvar[r][i],1) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddrvar = [[get_derivative_control_points_gurobi(self.robots_rvar[r][i],2) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_dhvar = [[get_derivative_control_points_gurobi(self.robots_hvar[r][i],1) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddhvar = [[get_derivative_control_points_gurobi(self.robots_hvar[r][i],2) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]

        self.objects_drvar = [[get_derivative_control_points_gurobi(self.objects_rvar[o][i],1) for i in range(self.objects_nbzs[o])] for o in range(len(self.objects))]
        self.objects_ddrvar = [[get_derivative_control_points_gurobi(self.objects_rvar[o][i],2) for i in range(self.objects_nbzs[o])] for o in range(len(self.objects))]
        self.objects_dhvar = [[get_derivative_control_points_gurobi(self.objects_hvar[o][i],1) for i in range(self.objects_nbzs[o])] for o in range(len(self.objects))]
        self.objects_ddhvar = [[get_derivative_control_points_gurobi(self.objects_hvar[o][i],2) for i in range(self.objects_nbzs[o])] for o in range(len(self.objects))]

        self.cost_expression = 0*self.robots_rvar[0][0][0,0]
        self.rho_phi = self.prog.addVar(lb=0,ub=self.bigM,vtype=gp.GRB.CONTINUOUS)

        self._continuity_constraints()
        self._initial_final_position_constraints()
        self._initial_final_velocity_constraints()
        self._initial_final_time_constraints()
        self._positive_time_derivative_constraint()
        self._velocity_constraints()
        self._world_box_constraints()
        if self.world.objects is not []:
            self._object_dynamics()
        else:
            self._robot_dynamics()

        # self._obstacle_collision_constraints()    # can now do this as an STL spec!!
        self._stl_constraints()

        # self._robot_robot_collision_constraints()
        # self._object_object_collision_constraints()

        # custom constraints for the video
        for r in range(self.nrobots):
            if self.robots[r].name == "crackle":
                # constrain all x points to be less than 2
                for bz in range(self.robots_nbzs[r]):
                    for cp in range(self.robots_ncp[r]):
                        self.prog.addConstr(self.robots_rvar[r][bz][0,cp] <= 2, name=f"crackle_x_{bz}_{cp}")

                    if bz == 1:
                        self.prog.addConstr(self.robots_rvar[r][bz][0,-1] == 0.5)
                        self.prog.addConstr(self.robots_rvar[r][bz][1,-1] == 0)
            if self.robots[r].name == "pop":
                # constrain all x points to be greater than 2
                for bz in range(self.robots_nbzs[r]):
                    for cp in range(self.robots_ncp[r]):
                        self.prog.addConstr(self.robots_rvar[r][bz][0,cp] >= 2, name=f"pop_x_{bz}_{cp}")

        self._set_cost()

    

    def solve(self):
        self.prog.setObjective(self.cost_expression,gp.GRB.MINIMIZE)
        
        t0 = time.time()
        self.prog.optimize()
        print(f"Optimization took {time.time()-t0} seconds")
        print(f"Code: {scs[self.prog.status]}")

        if self.prog.status is not gp.GRB.OPTIMAL:
            self.prog.computeIIS()
            print("IIS:")
            for c in self.prog.getConstrs():
                if c.IISConstr: print(f'\t{c.constrname}: {self.prog.getRow(c)} {c.Sense} {c.RHS}')


    def _set_cost(self):
        # # weights for throw_and_catch
        # weight_L = 0
        # weight_V = 0
        # weight_A = 0.51 
        # weight_absA = 0
        # weight_rho = 100000 #100000
        # weight_N_impacts = 10

        # # weights for lab_test_2
        # weight_L = 0
        # weight_V = 0
        # weight_A = 1000
        # weight_absA = 0
        # weight_rho = 1000 #100000
        # weight_N_impacts = 1

        # weights for complex stl spec
        weight_L = 0
        weight_V = 0
        weight_A = 0.1
        weight_absA = 0
        weight_rho = 10000 #100000
        weight_N_impacts = 0
        
        ### path length cost
        if weight_L > 0:
            try:
                for r in range(self.nrobots):
                    for bz in range(self.robots_nbzs[r]):
                        for cp in range(self.robots_ncp[r]-1):
                            for d in range(self.world.dim):
                                delta = self.prog.addVar(vtype=gp.GRB.CONTINUOUS)
                                self.cost_expression += delta

                                self.prog.addConstr(delta*delta >= \
                                                    weight_L*self.robots_drvar[r][bz][d,cp]*self.robots_drvar[r][bz][d,cp])
            except Exception as e:
                print(f"Exception objL: {e}")

        ### path velocity cost
        if weight_V > 0:
            try:
                for r in range(self.nrobots):
                    for bz in range(self.robots_nbzs[r]):
                        for cp in range(self.robots_ncp[r]-1):
                            for d in range(self.world.dim):
                                delta = self.prog.addVar(vtype=gp.GRB.CONTINUOUS)
                                self.cost_expression += delta

                                self.prog.addConstr(delta*(self.robots_order[r]*self.robots_dhvar[r][bz][0,cp]) >= \
                                                    weight_V*self.robots_drvar[r][bz][d,cp]*self.robots_drvar[r][bz][d,cp])
            except Exception as e:
                print(f"Exception objV: {e}")

        ### path acceleration cost
        if weight_A > 0:
            try:
                for r in range(self.nrobots):
                    for bz in range(self.robots_nbzs[r]):
                        for cp in range(self.robots_ncp[r]-2):
                            for d in range(self.world.dim):
                                attenuation_term = (2*weight_A)/(1+self.robots_order[r]-2)
                                self.cost_expression += attenuation_term*self.robots_ddrvar[r][bz][d,cp]*self.robots_ddrvar[r][bz][d,cp]
                                self.cost_expression += attenuation_term*self.robots_ddhvar[r][bz][0,cp]*self.robots_ddhvar[r][bz][0,cp]
            except Exception as e:
                print(f"Exception objU: {e}")

        ### linear path jerk cost approximation
        if weight_absA > 0:
            try:
                abs_ddrvars = []
                abs_ddhvars = []
                for r in range(self.nrobots):
                    for bz in range(self.robots_nbzs[r]):
                        for pi in range(self.n_parallel):
                            for cp in range(self.robots_ncp[r]-2):
                                for d in range(self.world.dim):
                                    ddrvar = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=-self.bigM, ub=self.bigM)
                                    self.prog.addConstr(ddrvar == self.robots_ddrvar[r][bz][d,cp,pi])
                                    abs_ddrvar = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=0, ub=self.bigM)
                                    self.prog.addConstr(abs_ddrvar == gp.abs_(ddrvar))
                                    abs_ddrvars.append(abs_ddrvar)

                                ddhvar = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=-self.bigM, ub=self.bigM)
                                self.prog.addConstr(ddhvar == self.robots_ddhvar[r][bz][0,cp,pi])
                                abs_ddhvar = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=0, ub=self.bigM)
                                self.prog.addConstr(abs_ddhvar == gp.abs_(ddhvar))
                                abs_ddhvars.append(abs_ddhvar)
                cost_var = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=0, ub=10)
                self.prog.addConstr(cost_var == gp.max_(abs_ddrvars))
                self.cost_expression += weight_absA*cost_var
            except Exception as e:
                print(f"Exception objabsU: {e}")

        ### STL space robustness cost
        if weight_rho > 0:
            self.cost_expression += -weight_rho*self.rho_phi

        ### number of impacts cost
        if weight_N_impacts > 0:
            try:
                z_sum = self.prog.addVar(vtype=gp.GRB.INTEGER)
                self.prog.addConstr(z_sum == gp.quicksum([self.robots[r].zs[o][bz,bzo]  
                                                          for r in range(self.nrobots) 
                                                          for o in range(self.nobjects)
                                                          for bz in range(self.robots_nbzs[r]-1) 
                                                          for bzo in range(self.objects_nbzs[o]-1)]))
                # for r in range(self.nrobots):
                #     for bz in range(self.robots_nbzs[r]-1):
                #         for o in range(self.nobjects):
                #             for bzo in range(self.objects_nbzs[o]-1):
                #                 self.cost_expression += weight_N_impacts*self.robots[r].zs[o][bz,bzo]
                self.cost_expression += weight_N_impacts*z_sum
            except Exception as e:
                print(f"Exception objN: {e}")

    def _continuity_constraints(self):
        for r in range(self.nrobots):
            for bz in range(self.robots_nbzs[r]-1):
                self.prog.addConstrs((self.robots_rvar[r][bz][i,-1] == self.robots_rvar[r][bz+1][i,0] for i in range(self.world.dim)), name=f"continuity_{r}_{bz}")
                self.prog.addConstr(self.robots_hvar[r][bz][0,-1] == self.robots_hvar[r][bz+1][0,0], name=f"continuity_{r}_{bz}_time")

        for o in range(self.nobjects):
            for bz in range(self.objects_nbzs[o]-1):
                self.prog.addConstrs((self.objects_rvar[o][bz][i,-1] == self.objects_rvar[o][bz+1][i,0] for i in range(self.world.dim)), name=f"continuity_{o}_{bz}")
                self.prog.addConstr(self.objects_hvar[o][bz][0,-1] == self.objects_hvar[o][bz+1][0,0], name=f"continuity_{o}_{bz}_time")
             
    def _initial_final_position_constraints(self):
        for r in range(self.nrobots):
            try:
                self.prog.addConstrs((self.robots_rvar[r][0][i,0]==self.robots[r].x0[i] for i in range(self.world.dim)), name=f"initial_position_robot_{r}")
                self.prog.addConstrs((self.robots_rvar[r][-1][i,-1]==self.robots[r].xf[i] for i in range(self.world.dim)), name=f"final_position_robot_{r}")
            except Exception as e:
                print(f"Error in {r} _initial_final_position_constraints: {e}")

        for o in range(self.nobjects):
            try:
                self.prog.addConstrs((self.objects_rvar[o][0][i,0]==self.objects[o].x0[i] for i in range(self.world.dim)), name=f"initial_position_object_{o}")
                if self.objects[o].xf is not None:
                    self.prog.addConstrs((self.objects_rvar[o][-1][i,-1]==self.objects[o].xf[i] for i in range(self.world.dim)), name=f"final_position_{o}")
                # self.prog.addConstrs((self.objects_rvar[o][-1][i,-1]==self.objects[o].xf[i] for i in range(self.world.dim)), name=f"final_position_object_{o}")
            except Exception as e:
                print(f"Error in {o} _initial_final_position_constraints: {e}")

    def _initial_final_velocity_constraints(self):
        for r in range(self.nrobots):
            try:
                for i in range(self.world.dim):
                    self.prog.addConstr(self.robots_drvar[r][0][i,0]==self.robots_dhvar[r][0][0,0]*self.robots[r].dx0[i], name=f"initial_velocity_{r}_{i}")
                    self.prog.addConstr(self.robots_drvar[r][-1][i,-1]==self.robots_dhvar[r][-1][0,-1]*self.robots[r].dxf[i], name=f"final_velocity_{r}_{i}")
            except Exception as e:
                print(f"Error in {r} _initial_final_velocity_constraints: {e}")

        for o in range(self.nobjects):
            try:
                for i in range(self.world.dim):
                    self.prog.addConstr(self.objects_drvar[o][0][i,0]==self.objects_dhvar[o][0][0,0]*self.objects[o].dx0[i], name=f"initial_velocity_{o}_{i}")
                    if self.objects[o].dxf is not None:
                        self.prog.addConstr(self.objects_drvar[o][-1][i,-1]==self.objects_dhvar[o][-1][0,-1]*self.objects[o].dxf[i], name=f"final_velocity_{o}_{i}")
                    # self.prog.addConstr(self.objects_drvar[o][-1][i,-1]==self.objects_dhvar[o][-1][0,-1]*self.objects[o].dxf[i], name=f"final_velocity_{o}_{i}")
            except Exception as e:
                print(f"Error in {o} _initial_final_velocity_constraints: {e}")

    def _initial_final_time_constraints(self):
        for r in range(self.nrobots):
            try:
                self.prog.addConstr(self.robots_hvar[r][0][0,0]==self.world.spec.t0, name=f"initial_time_{r}")
                self.prog.addConstr(self.robots_hvar[r][-1][0,-1]==self.world.spec.tf, name=f"final_time_{r}")
            except Exception as e:
                print(f"Error in {r} _initial_final_time_constraints: {e}")
    
        for o in range(self.nobjects):
            try:
                self.prog.addConstr(self.objects_hvar[o][0][0,0]==self.world.spec.t0, name=f"initial_time_{o}")
                self.prog.addConstr(self.objects_hvar[o][-1][0,-1]==self.world.spec.tf, name=f"final_time_{o}")
            except Exception as e:
                print(f"Error in {o} _initial_final_time_constraints: {e}")

    def _positive_time_derivative_constraint(self):
        for r in range(self.nrobots):
            for bz in range(self.robots_nbzs[r]):
                for cp in range(self.robots_ncp[r]-1):
                    try:
                        self.prog.addConstr(self.robots_dhvar[r][bz][0,cp]>=self.dh_lb, name=f"positive_time_derivative_{r}_{bz}_{cp}")
                        self.prog.addConstr(self.robots_dhvar[r][bz][0,cp]<=self.dh_ub, name=f"positive_time_derivative_{r}_{bz}_{cp}")
                    except Exception as e:
                        print(f"Error in {r} {bz} {cp} _positive_time_derivative_constraint: {e}")

        for o in range(self.nobjects):
            for bz in range(self.objects_nbzs[o]):
                for cp in range(self.objects_ncp[o]-1):
                    try:
                        self.prog.addConstr(self.objects_dhvar[o][bz][0,cp]>=self.dh_lb, name=f"positive_time_derivative_{o}_{bz}_{cp}")
                        self.prog.addConstr(self.objects_dhvar[o][bz][0,cp]<=self.dh_ub, name=f"positive_time_derivative_{o}_{bz}_{cp}")
                    except Exception as e:
                        print(f"Error in {o} {bz} {cp} _positive_time_derivative_constraint: {e}")

    def _velocity_constraints(self):
        for r in range(self.nrobots):
            for bz in range(self.robots_nbzs[r]):
                for cp in range(self.robots_ncp[r]-1):
                    try:
                        self.prog.addConstrs((self.robots_drvar[r][bz][i,cp]<=self.robots_dhvar[r][bz][0,cp]*self.robots[r].dq_ub[i] for i in range(self.world.dim)),
                                             name=f"velocity_{r}_{bz}_{cp}_1")
                        self.prog.addConstrs((self.robots_drvar[r][bz][i,cp]>=self.robots_dhvar[r][bz][0,cp]*self.robots[r].dq_lb[i] for i in range(self.world.dim)),
                                             name=f"velocity_{r}_{bz}_{cp}_2")
                    except Exception as e:
                        print(f"Error in {r} {bz} {cp} _velocity_constraints: {e}")

        for o in range(self.nobjects):
            for bz in range(self.objects_nbzs[o]):
                for cp in range(self.objects_ncp[o]-1):
                    try:
                        self.prog.addConstrs((self.objects_drvar[o][bz][i,cp]<=self.objects_dhvar[o][bz][0,cp]*self.objects[o].dq_ub[i] for i in range(self.world.dim)),
                                             name=f"velocity_{o}_{bz}_{cp}_1")
                        self.prog.addConstrs((self.objects_drvar[o][bz][i,cp]>=self.objects_dhvar[o][bz][0,cp]*self.objects[o].dq_lb[i] for i in range(self.world.dim)),
                                             name=f"velocity_{o}_{bz}_{cp}_2")
                    except Exception as e:
                        print(f"Error in {o} {bz} {cp} _velocity_constraints: {e}")

    def _world_box_constraints(self):
        for r in range(self.nrobots):
            for bz in range(self.robots_nbzs[r]):
                for cp in range(self.robots_ncp[r]):
                    try:
                        self.prog.addConstrs((self.robots_rvar[r][bz][i,cp]<=self.world.x_ub[i] for i in range(self.world.dim)), name=f"world_box_{r}_{bz}_{cp}_1")
                        self.prog.addConstrs((self.robots_rvar[r][bz][i,cp]>=self.world.x_lb[i] for i in range(self.world.dim)), name=f"world_box_{r}_{bz}_{cp}_2")
                    except Exception as e:
                        print(f"Error in {r} {bz} {cp} _world_box_constraints: {e}")
                    # self.prog.addConstr(self.robots_rvar[r][bz][0,cp]<=3, name=f"world_box_{r}_{bz}_{cp}_3")
                    # self.prog.addConstr(self.robots_rvar[r][bz][1,cp]<=1, name=f"world_box_{r}_{bz}_{cp}_3")

        for o in range(self.nobjects):
            for bz in range(self.objects_nbzs[o]):
                for cp in range(self.objects_ncp[o]):
                    try:
                        self.prog.addConstrs((self.objects_rvar[o][bz][i,cp]<=self.world.x_ub[i] for i in range(self.world.dim)), name=f"world_box_{o}_{bz}_{cp}_1")
                        self.prog.addConstrs((self.objects_rvar[o][bz][i,cp]>=self.world.x_lb[i] for i in range(self.world.dim)), name=f"world_box_{o}_{bz}_{cp}_2")
                    except Exception as e:
                        print(f"Error in {o} {bz} {cp} _world_box_constraints: {e}")
    
    def _object_dynamics(self):
        # the object may not be actuated by itself. It can only be pushed by the robots: elastic collision
        # for the object, the dynamics are: continuous velocity OR elastic collision with robot r
        # for the robot, the dynamics are: dynamics of r OR elastic collision with object o

        #! for a robot, for each bezier, we can bump with each object at each bezier or we are continuous
        #! for an object, for each bezier, we can bump with each robot at each bezier or we are continuous
        for r in range(self.nrobots):
            zs = [self.prog.addMVar((self.robots_nbzs[r]-1,self.objects_nbzs[o]-1),vtype=gp.GRB.BINARY) for o in range(self.nobjects)]
            self.robots[r].zs = zs
            #? for each robot bezier, we can only bump with one object bezier
            #? we can only bump with one object per robot bezier (is this implicitly constrained?)
            for bzr in range(self.robots_nbzs[r]-1):
                zs_sums = [self.prog.addVar(vtype=gp.GRB.BINARY) for o in range(self.nobjects)]
                for o in range(self.nobjects):
                    self.prog.addConstr(zs_sums[o] == gp.quicksum([zs[o][bzr,bzo] for bzo in range(self.objects_nbzs[o]-1)]))
                self.prog.addConstr(gp.quicksum(zs_sums)<=1)
            
            #? for each robot, two consecutive bzs can never both bump
            #? so we constrain that gp.quicksum([zs[o][bzr,:] for o in range(self.nobjects)]) <= 1
            for bzr in range(self.robots_nbzs[r]-1):
                try:
                    self.prog.addConstr(gp.quicksum([zs[o][bzr,bzo] for o in range(self.nobjects) for bzo in range(self.objects_nbzs[o]-1)]) <= 1)
                except Exception as e:
                    print(f"Error in {r} {bzr} [1,0,1,0] constriant: {e}")


            for o in range(self.nobjects):
                for bzr in range(self.robots_nbzs[r]-1):
                    for bzo in range(self.objects_nbzs[o]-1):
                        # if collision, then rvar[r][bzr][:,-1] == rvar[o][bzo][:,-1]
                        # and after collision we get
                        # - drvar[r][bzr+1][:,0] = (mr - mo)/(mr + mo)*drvar[r][bzr][:,-1] + 2*mo/(mr + mo)*drvar[o][bzo][:,-1]
                        # - drvar[o][bzo+1][:,0] = 2*mr/(mr + mo)*drvar[r][bzr][:,-1] + (mo - mr)/(mr + mo)*drvar[o][bzo][:,-1]
                        # otherwise, we add velocity continuity constraints

                        # if collision
                        try:
                            # position and time should be matched between robot and object
                            self.prog.addConstrs((self.robots_rvar[r][bzr][d,-1] >= self.objects_rvar[o][bzo][d,-1] - self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim)),
                                                 name=f"collision_{r}_{bzr}_{o}_{bzo}_1")
                            self.prog.addConstrs((self.robots_rvar[r][bzr][d,-1] <= self.objects_rvar[o][bzo][d,-1] + self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim)),
                                                 name=f"collision_{r}_{bzr}_{o}_{bzo}_2")
                            self.prog.addConstr(self.robots_hvar[r][bzr][0,-1] >= self.objects_hvar[o][bzo][0,-1] - self.bigM*(1-zs[o][bzr,bzo]),
                                                 name=f"collision_{r}_{bzr}_{o}_{bzo}_3")
                            self.prog.addConstr(self.robots_hvar[r][bzr][0,-1] <= self.objects_hvar[o][bzo][0,-1] + self.bigM*(1-zs[o][bzr,bzo]),
                                                 name=f"collision_{r}_{bzr}_{o}_{bzo}_4")
                        except Exception as e:
                            print(f"Error in {o} {bzo} {r} {bzr} if collision constraint: {e}")

                        m1 = (self.robots[r].mass - self.objects[o].mass)/(self.robots[r].mass + self.objects[o].mass)
                        m2 = 2*self.objects[o].mass/(self.robots[r].mass + self.objects[o].mass)
                        m3 = 2*self.robots[r].mass/(self.robots[r].mass + self.objects[o].mass)
                        m4 = (self.objects[o].mass - self.robots[r].mass)/(self.robots[r].mass + self.objects[o].mass)

                        try:
                            # impact dynamics
                            # inelastic collision (depending on e)
                            self.prog.addConstrs(self.robots[r].mass*self.robots_drvar[r][bzr+1][d,0] + self.objects[o].mass*self.objects_drvar[o][bzo+1][d,0] >=
                                                 self.robots[r].mass*self.robots_drvar[r][bzr][d,-1] + self.objects[o].mass*self.objects_drvar[o][bzo][d,-1] - self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim))
                            self.prog.addConstrs(self.robots[r].mass*self.robots_drvar[r][bzr+1][d,0] + self.objects[o].mass*self.objects_drvar[o][bzo+1][d,0] <=
                                                 self.robots[r].mass*self.robots_drvar[r][bzr][d,-1] + self.objects[o].mass*self.objects_drvar[o][bzo][d,-1] + self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim))
                            self.prog.addConstr(self.robots[r].mass*self.robots_dhvar[r][bzr+1][0,0] + self.objects[o].mass*self.objects_dhvar[o][bzo+1][0,0] >=
                                                 self.robots[r].mass*self.robots_dhvar[r][bzr][0,-1] + self.objects[o].mass*self.objects_dhvar[o][bzo][0,-1] - self.bigM*(1-zs[o][bzr,bzo]))
                            self.prog.addConstr(self.robots[r].mass*self.robots_dhvar[r][bzr+1][0,0] + self.objects[o].mass*self.objects_dhvar[o][bzo+1][0,0] <=
                                                 self.robots[r].mass*self.robots_dhvar[r][bzr][0,-1] + self.objects[o].mass*self.objects_dhvar[o][bzo][0,-1] + self.bigM*(1-zs[o][bzr,bzo]))
                            
                            self.prog.addConstrs(self.robots_drvar[r][bzr+1][d,0] - self.objects_drvar[o][bzo+1][d,0] >= 
                                                 -self.e*(self.robots_drvar[r][bzr][d,-1] - self.objects_drvar[o][bzo][d,-1]) - self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim))
                            self.prog.addConstrs(self.robots_drvar[r][bzr+1][d,0] - self.objects_drvar[o][bzo+1][d,0] <=
                                                 -self.e*(self.robots_drvar[r][bzr][d,-1] - self.objects_drvar[o][bzo][d,-1]) + self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim))
                            self.prog.addConstr(self.robots_dhvar[r][bzr+1][0,0] - self.objects_dhvar[o][bzo+1][0,0] >=
                                                -self.e*(self.robots_dhvar[r][bzr][0,-1] - self.objects_dhvar[o][bzo][0,-1]) - self.bigM*(1-zs[o][bzr,bzo]))
                            self.prog.addConstr(self.robots_dhvar[r][bzr+1][0,0] - self.objects_dhvar[o][bzo+1][0,0] <=
                                                -self.e*(self.robots_dhvar[r][bzr][0,-1] - self.objects_dhvar[o][bzo][0,-1]) + self.bigM*(1-zs[o][bzr,bzo]))
                        except Exception as e:
                            print(f"Error in {o} {bzo} {r} {bzr} impact dynamics constraint: {e}")

        # for the object, we enforce continuity, only if zs[r][o][bzr,bzo] == 0 for all r
        for o in range(self.nobjects):
            for bzo in range(self.objects_nbzs[o]-1):
                zs = [self.prog.addVar(vtype=gp.GRB.BINARY) for r in range(self.nrobots)]
                # zs[r] == max(self.robots[r].zs[r][o][:,bzo])
                for r in range(self.nrobots):
                    try:
                        self.prog.addConstr(zs[r] == gp.max_([self.robots[r].zs[o][bzr,bzo] for bzr in range(self.robots_nbzs[r]-1)]),
                                            name=f"zs_{o}_{bzo}_{r}")
                    except Exception as e:
                        print(f"Error in {o} {bzo} {r} zs[r] == max(robots[r].zs[r][o][:,bzo]): {e}")
                
                # if all zs are 0, then we enforce continuity
                zs_all = self.prog.addVar(vtype=gp.GRB.BINARY)
                try:
                    self.prog.addConstr(zs_all == gp.quicksum([zs[r] for r in range(self.nrobots)]),
                                        name=f"zs_all_{o}_{bzo}")
                except Exception as e:
                    print(f"Error in {o} {bzo} zs_all == quicksum(zs): {e}")

                # if zs_all == 0: continuity: tight constraints
                try:
                    self.prog.addConstrs((self.objects_drvar[o][bzo+1][d,0] >= self.objects_drvar[o][bzo][d,-1] - self.bigM*zs_all for d in range(self.world.dim)),
                                         name=f"continuity_{o}_{bzo}_1")
                    self.prog.addConstrs((self.objects_drvar[o][bzo+1][d,0] <= self.objects_drvar[o][bzo][d,-1] + self.bigM*zs_all for d in range(self.world.dim)),
                                         name=f"continuity_{o}_{bzo}_2")
                    self.prog.addConstr(self.objects_dhvar[o][bzo+1][0,0] >= self.objects_dhvar[o][bzo][0,-1] - self.bigM*zs_all,
                                         name=f"continuity_{o}_{bzo}_3")
                    self.prog.addConstr(self.objects_dhvar[o][bzo+1][0,0] <= self.objects_dhvar[o][bzo][0,-1] + self.bigM*zs_all,
                                         name=f"continuity_{o}_{bzo}_4")
                except Exception as e:
                    print(f"Error in {o} {bzo} continuity constraint: {e}")

        # for the robot, we enforce continuity, only if zs[r][o][bzr,bzo] == 0 for all bzo 
        for r in range(self.nrobots):
            for bzr in range(self.robots_nbzs[r]-1):
                zs = [self.prog.addVar(vtype=gp.GRB.BINARY) for o in range(self.nobjects)]
                # zs[o] == max(self.robots[r].zs[r][o][:,bzo])
                for o in range(self.nobjects):
                    try:
                        self.prog.addConstr(zs[o] == gp.max_([self.robots[r].zs[o][bzr,bzo] for bzo in range(self.objects_nbzs[o]-1)]),
                                            name=f"zs_{r}_{bzr}_{o}")
                    except Exception as e:
                        print(f"Error in {r} {bzr} {o} zs[o] == max(robots[r].zs[r][o][:,bzo]): {e}")
                
                # if all zs are 0, then we enforce continuity
                zs_all = self.prog.addVar(vtype=gp.GRB.BINARY)
                try:
                    self.prog.addConstr(zs_all == gp.quicksum([zs[o] for o in range(self.nobjects)]),
                                        name=f"zs_all_{r}_{bzr}")
                except Exception as e:
                    print(f"Error in {r} {bzr} zs_all == quicksum(zs): {e}")

                # if zs_all == 0: continuity: tight constraints
                try:
                    self.prog.addConstrs((self.robots_drvar[r][bzr+1][d,0] >= self.robots_drvar[r][bzr][d,-1] - self.bigM*zs_all for d in range(self.world.dim)),
                                         name=f"continuity_{r}_{bzr}_1")
                    self.prog.addConstrs((self.robots_drvar[r][bzr+1][d,0] <= self.robots_drvar[r][bzr][d,-1] + self.bigM*zs_all for d in range(self.world.dim)),
                                         name=f"continuity_{r}_{bzr}_2")
                    self.prog.addConstr(self.robots_dhvar[r][bzr+1][0,0] >= self.robots_dhvar[r][bzr][0,-1] - self.bigM*zs_all,
                                         name=f"continuity_{r}_{bzr}_3")
                    self.prog.addConstr(self.robots_dhvar[r][bzr+1][0,0] <= self.robots_dhvar[r][bzr][0,-1] + self.bigM*zs_all,
                                         name=f"continuity_{r}_{bzr}_4")
                except Exception as e:
                    print(f"Error in {r} {bzr} continuity constraint: {e}")
    
    def _robot_dynamics(self):
        # so apparently there are no obstacles, so all robots are just having smooth trajectories
        for r in range(self.nrobots):
            for bzr in range(self.robots_nbzs[r]-1):
                try:
                    self.prog.addConstrs((self.robots_drvar[r][bzr+1][d,0] == self.robots_drvar[r][bzr][d,-1] for d in range(self.world.dim)),
                                         name=f"robot_{r}_{bzr}_1")
                    self.prog.addConstrs((self.robots_dhvar[r][bzr+1][0,0] == self.robots_dhvar[r][bzr][0,-1] for d in range(self.world.dim)),
                                         name=f"robot_{r}_{bzr}_2")
                except Exception as e:
                    print(f"Error in {r} {bzr} robot dynamics constraint: {e}")

    def _obstacle_collision_constraints(self):
        for obs in self.world.obstacles:
            # for each robot
            for r in range(self.nrobots):
                for bzr in range(self.robots_nbzs[r]):
                    # desire that all control points in r lay outside atleast one face of the obstacle
                    # for each face of the obstacle
                    bs = self.prog.addMVar((obs.nfaces,),vtype=gp.GRB.BINARY)
                    for face in range(obs.nfaces):
                        for cp in range(self.robots_ncp[r]):
                            # for each control point of the robot
                            try:
                                ineqs = obs.H@self.robots_rvar[r][bzr][:,cp]
                                c = -ineqs[face] + obs.b[face]
                                self.prog.addConstr(c <= self.bigM*(1-bs[face]))
                            except Exception as e:
                                print(f"Error in obstacle collision constraint: {e}")

                    quicksum = self.prog.addVar(vtype=gp.GRB.BINARY)
                    self.prog.addConstr(quicksum == gp.quicksum([b for b in bs]))
                    self.prog.addConstr(quicksum >= 1)
                
            # for each object
            for o in range(self.nobjects):
                for bzo in range(self.objects_nbzs[o]):
                    # desire that all control points in o lay outside atleast one face of the obstacle
                    # for each face of the obstacle
                    bs = self.prog.addMVar((obs.nfaces,),vtype=gp.GRB.BINARY)
                    for face in range(obs.nfaces):
                        for cp in range(self.objects_ncp[o]):
                            # for each control point of the object
                            try:
                                ineqs = obs.H@self.objects_rvar[o][bzo][:,cp]
                                c = -ineqs[face] + obs.b[face]
                                self.prog.addConstr(c <= self.bigM*(1-bs[face]))
                            except Exception as e:
                                print(f"Error in obstacle collision constraint: {e}")

                    quicksum = self.prog.addVar(vtype=gp.GRB.BINARY)
                    self.prog.addConstr(quicksum == gp.quicksum([b for b in bs]))
                    self.prog.addConstr(quicksum >= 1)

    def _robot_robot_collision_constraints(self):
        checked_pairs = []
        self.z_times = []
        self.z_spaces = []
        for r1 in range(self.nrobots):
            for r2 in range(self.nrobots):
                if r1 is not r2 and (r1,r2) not in checked_pairs and (r2,r1) not in checked_pairs:
                    checked_pairs.append((r1,r2))
                    # loop through all beziers and if they intersect in time,
                    # add a constraint that they should not intersect in space
                    for bzr1 in range(self.robots_nbzs[r1]):
                        for bzr2 in range(self.robots_nbzs[r2]):
                            try:
                                z_time = parse_time_robot_robot(self,self.robots_hvar[r1][bzr1],\
                                                                self.robots_hvar[r2][bzr2])
                                # now we create 4 binary variables as we have 4 faces to stay out of 
                                z_space = self.prog.addMVar((self.world.dim**2,),vtype=gp.GRB.BINARY)
                                # we constrain that all control points of bzr1 should be left or right of all 
                                # control points of bzr2
                                for cp1 in range(self.robots_ncp[r1]):
                                    for cp2 in range(self.robots_ncp[r2]):
                                        # x cp1's left of x cp2's
                                        radius = 0.5
                                        try:
                                            c = self.robots_rvar[r1][bzr1][0,cp1] - self.robots_rvar[r2][bzr2][0,cp2] - radius
                                            self.prog.addConstr(c <= self.bigM*(1-z_space[0]) + self.bigM*(1-z_time))
                                        except Exception as e:
                                            print(f"Error in robot-robot collision constraint: {e}")
                                        # x cp1's right of x cp2's
                                        try:
                                            c = self.robots_rvar[r2][bzr2][0,cp2] - self.robots_rvar[r1][bzr1][0,cp1] - radius
                                            self.prog.addConstr(c <= self.bigM*(1-z_space[1]) + self.bigM*(1-z_time))
                                        except Exception as e:
                                            print(f"Error in robot-robot collision constraint: {e}")
                                        # y cp1's above of y cp2's
                                        try:
                                            c = self.robots_rvar[r1][bzr1][1,cp1] - self.robots_rvar[r2][bzr2][1,cp2] - radius
                                            self.prog.addConstr(c <= self.bigM*(1-z_space[2]) + self.bigM*(1-z_time))
                                        except Exception as e:
                                            print(f"Error in robot-robot collision constraint: {e}")
                                        # y cp1's below of y cp2's
                                        try:
                                            c = self.robots_rvar[r2][bzr2][1,cp2] - self.robots_rvar[r1][bzr1][1,cp1] - radius
                                            self.prog.addConstr(c <= self.bigM*(1-z_space[3]) + self.bigM*(1-z_time))
                                        except Exception as e:
                                            print(f"Error in robot-robot collision constraint: {e}")
                                # we constrain that atleast one of the 4 binary variables should be 1
                                quicksum = self.prog.addVar(vtype=gp.GRB.BINARY)
                                self.prog.addConstr(quicksum == gp.quicksum([z for z in z_space]))
                                self.prog.addConstr(quicksum >= 1)

                                self.z_times.append(z_time)
                                self.z_spaces.append(z_space)
                            except Exception as e:
                                print(f"Error in robot-robot collision constraint: {e}")

    def _object_object_collision_constraints(self):
        checked_pairs = []
        self.z_times = []
        self.z_spaces = []
        for o1 in range(self.nobjects):
            for o2 in range(self.nobjects):
                if o1 is not o2 and (o1,o2) not in checked_pairs and (o2,o1) not in checked_pairs:
                    checked_pairs.append((o1,o2))
                    # loop through all beziers and if they intersect in time,
                    # add a constraint that they should not intersect in space
                    for bzo1 in range(self.objects_nbzs[o1]):
                        for bzo2 in range(self.objects_nbzs[o2]):
                            try:
                                z_time = parse_time_robot_robot(self,self.objects_hvar[o1][bzo1],\
                                                                self.objects_hvar[o2][bzo2])
                                # now we create 4 binary variables as we have 4 faces to stay out of 
                                z_space = self.prog.addMVar((self.world.dim**2,),vtype=gp.GRB.BINARY)
                                # we constrain that all control points of bzo1 should be left or right of all 
                                # control points of bzo2
                                for cp1 in range(self.objects_ncp[o1]):
                                    for cp2 in range(self.objects_ncp[o2]):
                                        # x cp1's left of x cp2's
                                        radius = 0.5
                                        try:
                                            c = self.objects_rvar[o1][bzo1][0,cp1] - self.objects_rvar[o2][bzo2][0,cp2] + radius
                                            self.prog.addConstr(c <= self.bigM*(1-z_space[0]) + self.bigM*(1-z_time))
                                        except Exception as e:
                                            print(f"Error in object-object collision constraint: {e}")
                                        # x cp1's right of x cp2's
                                        try:
                                            c = self.objects_rvar[o2][bzo2][0,cp2] - self.objects_rvar[o1][bzo1][0,cp1] + radius
                                            self.prog.addConstr(c <= self.bigM*(1-z_space[1]) + self.bigM*(1-z_time))
                                        except Exception as e:
                                            print(f"Error in object-object collision constraint: {e}")
                                        # y cp1's above of y cp2's
                                        try:
                                            c = self.objects_rvar[o1][bzo1][1,cp1] - self.objects_rvar[o2][bzo2][1,cp2] + radius
                                            self.prog.addConstr(c <= self.bigM*(1-z_space[2]) + self.bigM*(1-z_time))
                                        except Exception as e:
                                            print(f"Error in object-object collision constraint: {e}")
                                        # y cp1's below of y cp2's
                                        try:
                                            c = self.objects_rvar[o2][bzo2][1,cp2] - self.objects_rvar[o1][bzo1][1,cp1] + radius
                                            self.prog.addConstr(c <= self.bigM*(1-z_space[3]) + self.bigM*(1-z_time))
                                        except Exception as e:
                                            print(f"Error in object-object collision constraint: {e}")
                                # we constrain that atleast one of the 4 binary variables should be 1
                                quicksum = self.prog.addVar(vtype=gp.GRB.BINARY)
                                self.prog.addConstr(quicksum == gp.quicksum([z for z in z_space]))
                                self.prog.addConstr(quicksum >= 1)

                                self.z_times.append(z_time)
                                self.z_spaces.append(z_space)
                            except Exception as e:
                                print(f"Error in object-object collision constraint: {e}")                

                                

    def _stl_constraints(self):
        try:
            for spec, name in zip(self.world.spec.preds,self.world.spec.names):
                print(f"\n\n Looking at spec of {name}")
                # find if name is a robot or object and which index
                hvar, rvar, nbzs, ncp = None, None, None, None
                for idx in range(self.nrobots):
                    if self.robots[idx].name == name:
                        hvar, rvar = self.robots_hvar[idx], self.robots_rvar[idx]
                        nbzs, ncp = self.robots_nbzs[idx], self.robots_ncp[idx]
                        break
                for idx in range(self.nobjects):
                    if self.objects[idx].name == name:
                        hvar, rvar = self.objects_hvar[idx], self.objects_rvar[idx]
                        nbzs, ncp = self.objects_nbzs[idx], self.objects_ncp[idx]
                        break
                robot_vars = {'hvar': hvar, 'rvar': rvar, 'nbzs': nbzs, 'ncp': ncp}
                quant_parse_operator(self,spec,robot_vars,self.rho_phi)
                self.prog.addConstr(spec.z == 1)
            # self.prog.addConstr(self.rho_phi == self.world.spec.preds[0].rho)
        except Exception as e:
            print(f"No STL constraints! {e}")

    def write_ids_and_names(self):
        # for the robots
        for r in range(self.nrobots):
            for o in range(self.nobjects):
                zs_sol = self.robots[r].zs[o].X
                print(f"Robot {r} Object {o} zs: \n{zs_sol}")
                for bzr in range(self.robots[r].nbz-1):
                    if any(zs_sol[bzr,:] == 1):
                        self.robots[r].ids[bzr] = 'pre'
                        self.robots[r].other_names[bzr] = self.objects[o].name
                    if bzr > 0 and any(zs_sol[bzr-1,:] == 1):
                        self.robots[r].ids[bzr] = 'post'
                        self.robots[r].other_names[bzr] = self.objects[o].name
        # for the objects
        for o in range(self.nobjects):
            for r in range(self.nrobots):
                zs_sol = self.robots[r].zs[o].X
                print(f"Object {o} Robot {r} zs: \n{zs_sol}")
                for bzo in range(self.objects[o].nbz-1):
                    if any(zs_sol[:,bzo] == 1):
                        self.objects[o].ids[bzo] = 'pre'
                        self.objects[o].other_names[bzo] = self.robots[r].name
                    if bzo > 0 and any(zs_sol[:,bzo-1] == 1):
                        self.objects[o].ids[bzo] = 'post'
                        self.objects[o].other_names[bzo] = self.robots[r].name


    def evaluate(self):
        self.solve_status = self.prog.status
        self.obj_value = self.prog.ObjVal

        self.rho_sol = self.rho_phi.X
        print(f"rho: {self.rho_sol}")
        # loop through the specification and evaluate rho of each pred
        def eval_pred(pred):
            try:
                for p in pred.preds:
                    eval_pred(p)
            except:
                pass

            try:
                print(f"z of {pred.get_string()}: {pred.z.X}")
            except:
                print(f"zs of {pred.get_string()}: {pred.zs.X}")
            if pred.type == "F" or pred.type == "G":
                print(f"z_time of {pred.get_string()}: {pred.z_time.X}")
        try:
            for spec,name in zip(self.world.spec.preds,self.world.spec.names):
                print(f"\n\n Looking at spec of {name}")
                eval_pred(spec)
        except Exception as e:
            print(f"No STL constraints! {e}")

        # evaluate collision avoidance binary variables if exists
        if hasattr(self,'z_times'):
            for z_time in self.z_times:
                print(f"z_time: {z_time.X}")
            for z_space in self.z_spaces:
                print(f"z_space: {z_space.X}")

        self.write_ids_and_names()

        self.robots_rsol = [[self.robots_rvar[r][bz].X for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_drsol = [[get_derivative_control_points_gurobi(self.robots_rsol[r][bz],1) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddrsol = [[get_derivative_control_points_gurobi(self.robots_rsol[r][bz],2) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_hsol = [[self.robots_hvar[r][bz].X for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_dhsol = [[get_derivative_control_points_gurobi(self.robots_hsol[r][bz],1) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddhsol = [[get_derivative_control_points_gurobi(self.robots_hsol[r][bz],2) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_qsol = self.robots_rsol
        self.robots_dqsol = [[np.zeros_like(self.robots_drsol[r][bz]) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        for r in range(self.nrobots):
            for bz in range(self.robots_nbzs[r]):
                for cp in range(self.robots_ncp[r]-1):
                    self.robots_dqsol[r][bz][:,cp] = self.robots_drsol[r][bz][:,cp]/self.robots_dhsol[r][bz][0,cp]

        # now evaluate the bezier curves to obtain the trajectories
        self.N_eval = 100
        self.robots_rtraj = [[eval_bezier(self.robots_rsol[r][bz],self.N_eval) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_drtraj = [[eval_bezier(self.robots_drsol[r][bz],self.N_eval) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddrtraj = [[eval_bezier(self.robots_ddrsol[r][bz],self.N_eval) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_htraj = [[eval_bezier(self.robots_hsol[r][bz],self.N_eval) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_dhtraj = [[eval_bezier(self.robots_dhsol[r][bz],self.N_eval) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddhtraj = [[eval_bezier(self.robots_ddhsol[r][bz],self.N_eval) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_qtraj = self.robots_rtraj
        self.robots_dqtraj = [[np.zeros((self.world.dim,self.N_eval)) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddqtraj = [[np.zeros((self.world.dim,self.N_eval)) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        for r in range(self.nrobots):
            for bz in range(self.robots_nbzs[r]):
                for i in range(self.world.dim):
                    self.robots_dqtraj[r][bz][i,:] = self.robots_drtraj[r][bz][i,:]/self.robots_dhtraj[r][bz][0,:]
                    self.robots_ddqtraj[r][bz][i,:] = (self.robots_ddrtraj[r][bz][i,:]*self.robots_dhtraj[r][bz][0,:]- \
                                                       self.robots_drtraj[r][bz][i,:]*self.robots_ddhtraj[r][bz][0,:])/(self.robots_dhtraj[r][bz][0,:]**2)
        
        self.objects_rsol = [[self.objects_rvar[o][bz].X for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_drsol = [[get_derivative_control_points_gurobi(self.objects_rsol[o][bz],1) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_ddrsol = [[get_derivative_control_points_gurobi(self.objects_rsol[o][bz],2) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_hsol = [[self.objects_hvar[o][bz].X for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_dhsol = [[get_derivative_control_points_gurobi(self.objects_hsol[o][bz],1) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_ddhsol = [[get_derivative_control_points_gurobi(self.objects_hsol[o][bz],2) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_qsol = self.objects_rsol
        self.objects_dqsol = [[np.zeros_like(self.objects_drsol[o][bz]) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        for o in range(self.nobjects):
            for bz in range(self.objects_nbzs[o]):
                for cp in range(self.objects_ncp[o]-1):
                    self.objects_dqsol[o][bz][:,cp] = self.objects_drsol[o][bz][:,cp]/self.objects_dhsol[o][bz][0,cp]

        # now evaluate the bezier curves to obtain the trajectories
        #TODO: check ddrtraj for the obstacle, acceleration is not defined for the wait and float phases!
        self.objects_rtraj = [[eval_bezier(self.objects_rsol[o][bz],self.N_eval) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_drtraj = [[eval_bezier(self.objects_drsol[o][bz],self.N_eval) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        # self.objects_ddrtraj = [[eval_bezier(self.objects_ddrsol[o][bz],self.N_eval) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_htraj = [[eval_bezier(self.objects_hsol[o][bz],self.N_eval) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_dhtraj = [[eval_bezier(self.objects_dhsol[o][bz],self.N_eval) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        # self.objects_ddhtraj = [[eval_bezier(self.objects_ddhsol[o][bz],self.N_eval) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        self.objects_qtraj = self.objects_rtraj
        self.objects_dqtraj = [[np.zeros((self.world.dim,self.N_eval)) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        # self.objects_ddqtraj = [[np.zeros((self.world.dim,self.N_eval)) for bz in range(self.objects_nbzs[o])] for o in range(self.nobjects)]
        for o in range(self.nobjects):
            for bz in range(self.objects_nbzs[o]):
                for i in range(self.world.dim):
                    self.objects_dqtraj[o][bz][i,:] = self.objects_drtraj[o][bz][i,:]/self.objects_dhtraj[o][bz][0,:]
                    # self.objects_ddqtraj[o][bz][i,:] = (self.objects_ddrtraj[o][bz][i,:]*self.objects_dhtraj[o][bz][0,:]- \
                    #                                    self.objects_drtraj[o][bz][i,:]*self.objects_ddhtraj[o][bz][0,:])/(self.objects_dhtraj[o][bz][0,:]**2)

        # now save all the results of the robots to a .csv file
        from utilities.read_write_plan import plan_to_csv
        for r in range(self.nrobots):
            # append a row of zeros to robots_rsol for the theta dimension which we don't plan for
            robots_rsol = [np.vstack((self.robots_rsol[r][bz],np.zeros((1,self.robots_ncp[r])))) for bz in range(self.robots_nbzs[r])]
            plan_to_csv(robots_rsol,self.robots_hsol[r],self.robots[r].ids,self.robots[r].other_names,
                        scenario_name=self.world.specification,
                        robot_name=self.robots[r].name,
                        path='./impact_stl/impact_stl/planner/plans/')
        for o in range(self.nobjects):
            objects_rsol = [np.vstack((self.objects_rsol[o][bz],np.zeros((1,self.objects_ncp[o])))) for bz in range(self.objects_nbzs[o])]
            plan_to_csv(objects_rsol,self.objects_hsol[o],self.objects[o].ids,self.objects[o].other_names,
                        scenario_name=self.world.specification,
                        robot_name=self.objects[o].name,
                        path='./impact_stl/impact_stl/planner/plans/')
            
    def evaluate_t(self,t):
        t_array_robots = [np.array([self.robots_hsol[r][bzr][0,0] for bzr in range(self.robots_nbzs[r])]) for r in range(self.nrobots)]
        t_array_objects = [np.array([self.objects_hsol[o][bzo][0,0] for bzo in range(self.objects_nbzs[o])]) for o in range(self.nobjects)]

        idxs_robots = [np.where(t_array_robots[r]<=t)[0][-1] for r in range(self.nrobots)]
        idxs_objects = [np.where(t_array_objects[o]<=t)[0][-1] for o in range(self.nobjects)]

        errors_robots = []
        errors_objects = []
        for r in range(self.nrobots):
            errors_robots.append(lambda s: value_bezier(self.robots_htraj[r][idxs_robots[r]],s)[0] - t)
        for o in range(self.nobjects):
            errors_objects.append(lambda s: value_bezier(self.objects_htraj[o][idxs_objects[o]],s)[0] - t)

        s_robots = []
        s_objects = []
        for r in range(self.nrobots):
            s_robots.append(optimize.root_scalar(errors_robots[r],bracket=[0,1]).root)
        for o in range(self.nobjects):
            s_objects.append(optimize.root_scalar(errors_objects[o],bracket=[0,1]).root)

        return idxs_robots, s_robots, idxs_objects, s_objects


    def plot(self):
        # old layout for fig 4\
        fig, axs = plt.subplots(2,3,figsize=(15,10))
        ax1 = axs[0,0]
        ax2 = axs[0,1]
        ax3 = axs[0,2]
        ax4 = axs[1,0]
        ax5 = axs[1,1]
        ax6 = axs[1,2]

        # new layout for results figures
        fig = plt.figure(figsize=(20,10))
        gs = GridSpec(2,4, figure=fig)
        ax1 = fig.add_subplot(gs[0:2,0:2])
        ax2 = fig.add_subplot(gs[0,2:4])
        ax3 = fig.add_subplot(gs[1,2:4])
        # ax4 = fig.add_subplot(gs[1,2])
        # ax5 = fig.add_subplot(gs[1,3])

        robot_ls = ['k-','k--','k:','k-.']
        object_ls = ['r-','r--','r:','r-.']
        lw = 4
        s = 15
        if True:
            self.world.plot(ax1)
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    ax1.plot(self.robots_rtraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],robot_ls[r],linewidth=lw)
                    ax1.plot(self.robots_rtraj[r][bz][0,0],self.robots_rtraj[r][bz][1,0],'ko',markersize=s)
                ax1.plot(self.robots_rtraj[r][-1][0,-1],self.robots_rtraj[r][-1][1,-1],'ko',markersize=s)
            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    ax1.plot(self.objects_rtraj[o][bz][0,:],self.objects_rtraj[o][bz][1,:],object_ls[o],linewidth=lw)
                    ax1.plot(self.objects_rtraj[o][bz][0,0],self.objects_rtraj[o][bz][1,0],'ro',markersize=s)
                ax1.plot(self.objects_rtraj[o][-1][0,-1],self.objects_rtraj[o][-1][1,-1],'ro',markersize=s)
            # ax1.set_title("x-y plane")
            # ax1.set_xlim(0.25,4.0)
            # ax1.set_ylim(-1.25,1.75)
            # ax1.set_xticks(np.arange(0.5, 4, 0.5))
            # ax1.set_yticks(np.arange(-1.0, 2.0, 0.5))
            ax1.set_xlabel(r"x position [m]")
            ax1.set_ylabel(r"y position [m]")
            ax1.grid(True)
            ax1.set_aspect('equal', 'box')
        # if True:
        #     # self.world.plot(ax1)
        #     for r in range(self.nrobots):
        #         for bz in range(self.robots_nbzs[r]):
        #             ax1.plot(-self.robots_rtraj[r][bz][1,:],self.robots_rtraj[r][bz][0,:],robot_ls[r])
        #             ax1.plot(-self.robots_rtraj[r][bz][1,0],self.robots_rtraj[r][bz][0,0],'ko')
        #         ax1.plot(-self.robots_rtraj[r][-1][1,-1],self.robots_rtraj[r][-1][0,-1],'ko')
        #     for o in range(self.nobjects):
        #         for bz in range(self.objects_nbzs[o]):
        #             ax1.plot(-self.objects_rtraj[o][bz][1,:],self.objects_rtraj[o][bz][0,:],'r')
        #             ax1.plot(-self.objects_rtraj[o][bz][1,0],self.objects_rtraj[o][bz][0,0],'ro')
        #         ax1.plot(-self.objects_rtraj[o][-1][1,-1],self.objects_rtraj[o][-1][0,-1],'ro')
        #     # ax1.set_title("x-y plane")
        #     ax1.set_xlim(-0.5,1.0)
        #     ax1.set_ylim(0,3.5)
        #     ax1.set_xlabel(r"x position [m]")
        #     ax1.set_ylabel(r"y position [m]")
        #     ax1.grid(True)
        #     ax1.set_aspect('equal', 'box')

        if True:
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    ax2.plot(self.robots_htraj[r][bz][0,:],self.robots_rtraj[r][bz][0,:],robot_ls[r],linewidth=lw)
                    ax2.plot(self.robots_htraj[r][bz][0,0],self.robots_rtraj[r][bz][0,0],'ko',markersize=s)
                ax2.plot(self.robots_htraj[r][-1][0,-1],self.robots_rtraj[r][-1][0,-1],'ko',markersize=s)
            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    ax2.plot(self.objects_htraj[o][bz][0,:],self.objects_rtraj[o][bz][0,:],object_ls[o],linewidth=lw)
                    ax2.plot(self.objects_htraj[o][bz][0,0],self.objects_rtraj[o][bz][0,0],'ro',markersize=s)
                ax2.plot(self.objects_htraj[o][-1][0,-1],self.objects_rtraj[o][-1][0,-1],'ro',markersize=s)
            # ax2.set_title("x-t plane")
            ax2.grid(True)
            ax2.set_xlabel(r"Time [s]")
            ax2.set_ylabel(r"x position [m]")

        if True:
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    ax3.plot(self.robots_htraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],robot_ls[r],linewidth=lw)
                    ax3.plot(self.robots_htraj[r][bz][0,0],self.robots_rtraj[r][bz][1,0],'ko',markersize=s)
                ax3.plot(self.robots_htraj[r][-1][0,-1],self.robots_rtraj[r][-1][1,-1],'ko',markersize=s)
            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    ax3.plot(self.objects_htraj[o][bz][0,:],self.objects_rtraj[o][bz][1,:],object_ls[o],linewidth=lw)
                    ax3.plot(self.objects_htraj[o][bz][0,0],self.objects_rtraj[o][bz][1,0],'ro',markersize=s)
                ax3.plot(self.objects_htraj[o][-1][0,-1],self.objects_rtraj[o][-1][1,-1],'ro',markersize=s)
            # ax3.set_title("y-t plane")
            ax3.grid(True)
            ax3.set_xlabel(r"Time [s]")
            ax3.set_ylabel(r"y position [m]")

        if False:
            # plot h_values for robots and objects
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    axs[1,0].plot(self.robots_htraj[r][bz][0,:],robot_ls[r])
            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    axs[1,0].plot(self.objects_htraj[o][bz][0,:],'r')
            # axs[1,0].set_title("h-t plane")
            axs[1,0].set_xlabel(r"Time [s]")
            axs[1,0].set_ylabel(r"Phase [-]")

        # if True:
        #     for r in range(self.nrobots):
        #         for bz in range(self.robots_nbzs[r]):
        #             ax4.plot(self.robots_htraj[r][bz][0,:],self.robots_dqtraj[r][bz][0,:],robot_ls[r],linewidth=lw)
        #             ax4.plot(self.robots_htraj[r][bz][0,0],self.robots_dqtraj[r][bz][0,0],'ko',markersize=s)
        #         ax4.plot(self.robots_htraj[r][-1][0,-1],self.robots_dqtraj[r][-1][0,-1],'ko',markersize=s)
        #     for o in range(self.nobjects):
        #         for bz in range(self.objects_nbzs[o]):
        #             ax4.plot(self.objects_htraj[o][bz][0,:],self.objects_dqtraj[o][bz][0,:],object_ls[o],linewidth=lw)
        #             ax4.plot(self.objects_htraj[o][bz][0,0],self.objects_dqtraj[o][bz][0,0],'ro',markersize=s)
        #         ax4.plot(self.objects_htraj[o][-1][0,-1],self.objects_dqtraj[o][-1][0,-1],'ro',markersize=s)
        #     # ax4.set_title("dx-t plane")
        #     ax4.grid(True)
        #     ax4.set_xlabel(r"Time [s]")
        #     ax4.set_ylabel(r"y velocity [m/s]")

        # if True:
        #     for r in range(self.nrobots):
        #         for bz in range(self.robots_nbzs[r]):
        #             ax5.plot(self.robots_htraj[r][bz][0,:],self.robots_dqtraj[r][bz][1,:],robot_ls[r],linewidth=lw)
        #             ax5.plot(self.robots_htraj[r][bz][0,0],self.robots_dqtraj[r][bz][1,0],'ko',markersize=s)
        #         ax5.plot(self.robots_htraj[r][-1][0,-1],self.robots_dqtraj[r][-1][1,-1],'ko',markersize=s)
        #     for o in range(self.nobjects):
        #         for bz in range(self.objects_nbzs[o]):
        #             ax5.plot(self.objects_htraj[o][bz][0,:],self.objects_dqtraj[o][bz][1,:],object_ls[o],linewidth=lw)
        #             ax5.plot(self.objects_htraj[o][bz][0,0],self.objects_dqtraj[o][bz][1,0],'ro',markersize=s)
        #         ax5.plot(self.objects_htraj[o][-1][0,-1],self.objects_dqtraj[o][-1][1,-1],'ro',markersize=s)
        #     # ax5.set_title("dy-t plane")
        #     ax5.grid(True)
        #     ax5.set_xlabel(r"Time [s]")
        #     ax5.set_ylabel(r"x velocity [m/s]")
        
        fig.tight_layout()
        
        plt.savefig("./impact_stl/impact_stl/planner/figures/plot.svg")
        plt.savefig("./impact_stl/impact_stl/planner/figures/plot.png")

        
    def animate(self):
        Neval = 250
        self.t_range = np.linspace(self.world.spec.t0,self.world.spec.tf,Neval)

        self.fig_anim = plt.figure(figsize=(10,10))
        self.ax_anim = plt.axes()
        anim = animation.FuncAnimation(self.fig_anim,self._animate_update,
                                        frames=Neval,interval=self.world.spec.tf/Neval*1e6*2)
        anim.save("./impact_stl/impact_stl/planner/figures/animation.mp4",writer="ffmpeg", fps=Neval/self.world.spec.tf)

    def _animate_update(self,i):
        t = self.t_range[i]
        idxs_robots, s_robots, idxs_objects, s_objects = self.evaluate_t(t)

        self.ax_anim.clear()
        self.world.plot(self.ax_anim)
        for r in range(self.nrobots):
            # plot circle
            c = (value_bezier(self.robots_rtraj[r][idxs_robots[r]],s_robots[r])[0],
                value_bezier(self.robots_rtraj[r][idxs_robots[r]],s_robots[r])[1])
            rad = 0.2
            circle = plt.Circle(c,rad,fill=False,color='k')
            self.ax_anim.add_patch(circle)
            # plot trajectory 
            for bz in range(self.robots_nbzs[r]):
                self.ax_anim.plot(self.robots_rtraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],'k')
        for o in range(self.nobjects):
            # plot circle
            c = (value_bezier(self.objects_rtraj[o][idxs_objects[o]],s_objects[o])[0],
                value_bezier(self.objects_rtraj[o][idxs_objects[o]],s_objects[o])[1])
            rad = 0.2
            circle = plt.Circle(c,rad,fill=False,color='r')
            self.ax_anim.add_patch(circle)
            # plot trajectory
            for bz in range(self.objects_nbzs[o]):
                self.ax_anim.plot(self.objects_rtraj[o][bz][0,:],self.objects_rtraj[o][bz][1,:],'r')
        self.ax_anim.set_title("x-y plane")
        self.ax_anim.set_xlabel("x [m]")
        self.ax_anim.set_ylabel("y [m]")
        self.ax_anim.set_aspect("equal")

    def plot_atmos(self):
        # old layout for fig 4\
        # new layout for results figures
        fig = plt.figure(figsize=(35,10))
        gs = GridSpec(1,5, figure=fig)
        ax1 = fig.add_subplot(gs[0,0:2])
        ax2 = fig.add_subplot(gs[0,2])
        ax3 = fig.add_subplot(gs[0,3])
        ax4 = fig.add_subplot(gs[0,4])

        to_save = [{} for _ in range(self.nrobots)]

        robot_ls = ['k','k--']
        if True:
            self.world.plot(ax1)
            for r in range(self.nrobots):
                to_save[r]['rtraj'] = self.robots_rtraj[r]
                to_save[r]['htraj'] = self.robots_htraj[r]
                for bz in range(self.robots_nbzs[r]):
                    ax1.plot(self.robots_rtraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],robot_ls[r])
                    ax1.plot(self.robots_rtraj[r][bz][0,0],self.robots_rtraj[r][bz][1,0],'ko')
                ax1.plot(self.robots_rtraj[r][-1][0,-1],self.robots_rtraj[r][-1][1,-1],'ko')
            # ax1.set_title("x-y plane")
            # ax1.set_xlim(0.25,4.0)
            # ax1.set_ylim(-1.25,1.75)
            # ax1.set_xticks(np.arange(0.5, 4, 0.5))
            # ax1.set_yticks(np.arange(-1.0, 2.0, 0.5))
            ax1.set_xlabel(r"x position [m]")
            ax1.set_ylabel(r"y position [m]")
            ax1.grid(True)
            ax1.set_aspect('equal', 'box')

        if True:
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    if bz == 0:
                        ax2.plot(self.robots_htraj[r][bz][0,:],self.robots_rtraj[r][bz][0,:],robot_ls[r],label=r'$p_x$')
                    else:
                        ax2.plot(self.robots_htraj[r][bz][0,:],self.robots_rtraj[r][bz][0,:],robot_ls[r])
                    ax2.plot(self.robots_htraj[r][bz][0,0],self.robots_rtraj[r][bz][0,0],'ko')
                ax2.plot(self.robots_htraj[r][-1][0,-1],self.robots_rtraj[r][-1][0,-1],'ko')
            # ax3.set_title("y-t plane")
            ax2.grid(True)
            ax2.set_ylim([0.0,3.0])
            ax2.set_xlabel(r"Time [s]")
            ax2.set_ylabel(r"x-position [m]")
            ax2.legend()

            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    if bz == 0:
                        ax3.plot(self.robots_htraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],robot_ls[r],label=r'$p_y$')
                    else:
                        ax3.plot(self.robots_htraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],robot_ls[r])
                    ax3.plot(self.robots_htraj[r][bz][0,0],self.robots_rtraj[r][bz][1,0],'ko')
                ax3.plot(self.robots_htraj[r][-1][0,-1],self.robots_rtraj[r][-1][1,-1],'ko')
            # ax3.set_title("y-t plane")
            ax3.grid(True)
            ax3.set_ylim([-1,1])
            ax3.set_xlabel(r"Time [s]")
            ax3.set_ylabel(r"y-position [m]")
            ax3.legend()

        if True:
            # compute STL robustness on robots_rtraj[r]
            for r in range(self.nrobots):
                in_preds = [self.world.spec.preds[0].preds[0].preds[0], 
                            self.world.spec.preds[0].preds[1].preds[0],
                            self.world.spec.preds[0].preds[2].preds[0],
                            self.world.spec.preds[0].preds[3].preds[0]]
                out_preds = [self.world.spec.preds[0].preds[4].preds[0]]
                # in_preds = [self.world.spec.preds[0].preds[0].preds[0], 
                #             self.world.spec.preds[0].preds[1].preds[0]]
                # out_preds = [self.world.spec.preds[0].preds[2].preds[0]]
                rho_in_preds = [np.zeros((self.N_eval*self.robots_nbzs[r],)) for _ in in_preds]
                rho_out_preds = [np.zeros((self.N_eval*self.robots_nbzs[r],)) for _ in out_preds]
                # create htraj which is all appended robots_htraj[r][bz] for each bz
                htraj = np.hstack([self.robots_htraj[r][bz] for bz in range(self.robots_nbzs[r])])
                rtraj = np.hstack([self.robots_rtraj[r][bz] for bz in range(self.robots_nbzs[r])])

                for pred,rho in zip(in_preds,rho_in_preds):
                    for i in range(self.N_eval*self.robots_nbzs[r]):
                        rho[i] = min(-pred.preds.H@rtraj[:,i] + pred.preds.b)
                for pred,rho in zip(out_preds,rho_out_preds):
                    for i in range(self.N_eval*self.robots_nbzs[r]):
                        rho[i] = max(pred.preds.H@rtraj[:,i] - pred.preds.b)

                # ax3.plot(htraj[0,:],rho_in_preds[0],'k',label=r"$\rho_{\diamondsuit_{[0,t_f]}(x \in A)}$")
                # ax3.plot(htraj[0,:],rho_in_preds[1],'k--',label=r"$\rho_{\diamondsuit_{[0,t_f]}(y \in B)}$")
                # ax3.plot(htraj[0,:],rho_out_preds[0],'r',label=r"$\rho_{\Box_{[0,t_f]}(x \notin Obs)}$")

                rho_phi = rho_out_preds[0]
                for rho in rho_in_preds:
                    rho_max, idx = np.max(rho), np.argmax(rho)
                    rho_phi[idx] = rho_max
                ax4.plot(htraj[0,:],rho_phi,'b',label=r"$\rho_{\phi}$")
                to_save[r]['rho_phi'] = rho_phi
            

            ax4.grid(True)
            ax4.set_ylim([0,1.8])
            ax4.set_xlabel(r"Time [s]")
            ax4.set_ylabel(r"$\rho_{\phi}$ [m]")
            # ax3.legend()
        
        fig.tight_layout()
        
        plt.savefig("./impact_stl/impact_stl/planner/figures/plot.svg")
        plt.savefig("./impact_stl/impact_stl/planner/figures/plot.png")

        # save to_save object
        with open('./impact_stl/impact_stl/planner/figures/to_save.pkl', 'wb') as f:
            pickle.dump(to_save, f)