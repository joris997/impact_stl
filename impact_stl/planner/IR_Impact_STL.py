import gurobipy as gp
# scs = {v: k for k, v in vars(gp.StatusConstClass).items() if k[0].isupper()}
scs = {getattr(gp.GRB.status,k): k for k in dir(gp.GRB.status) if k[0].isupper()}

import time
import numpy as np
import scipy as sp

# plotting imports
import scienceplots
import matplotlib.pyplot as plt
from matplotlib import rc
from matplotlib.axes import Axes
from matplotlib.patches import Patch
from matplotlib.gridspec import GridSpec
from matplotlib import animation
# Set up plot style
plt.style.use(['science'])
rc('text', usetex=True)
rc('font', family='times', size=20)

plt.rcParams['legend.frameon'] = True             # Enable legend frame
plt.rcParams['legend.facecolor'] = 'white'        # Set background color
plt.rcParams['legend.edgecolor'] = 'white'        # Set border color
plt.rcParams['legend.framealpha'] = 1.0
plt.rcParams['legend.loc'] = 'best'

from utilities.beziers import get_derivative_control_points_gurobi, get_derivative_control_points_parallel_gurobi,\
                              eval_bezier, value_bezier
from utilities.zonotopes import zonotope
from utilities.opt_helpers import createSupZonotope, addBilinearConstr, constrainZinZ, createBoundingZonotope, \
                                  constrainZoutZ, constrainRinH, constrainRoutH, constrainZoutH
from utilities.ir_stl import quant_parse_operator
from World import World
from Spec import Spec

class IR_Impact_STL:
    def __init__(self,world:World, spec:Spec=None):
        # load the STL specification, with x0 and xf for all robots and objects
        # self.spec = world.spec
        self.world = world
        self.robots = world.robots
        self.objects = world.objects

        self.bigM = 1e4
        self.dh_lb = 1e0 # was 1e-3
        self.dh_ub = 1e3

        self.spec = spec

    def construct(self):
        self.prog = gp.Model("impact_stl")
        # https://docs.gurobi.com/projects/optimizer/en/current/reference/parameters.html
        self.prog.setParam(gp.GRB.Param.OutputFlag, 1)
        # self.prog.setParam(gp.GRB.Param.MIPGap, 1e1)
        # self.prog.setParam(gp.GRB.Param.ImproveStartGap, 1e1)
        # self.prog.setParam(gp.GRB.Param.ConcurrentJobs, 6)
        # self.prog.setParam(gp.GRB.Param.LazyConstraints, 1)

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

        self.signs = []
        for i in range(2**(2*self.world.dim)):
            self.signs.append([(-1)**((i >> j) & 1) for j in range(4)])

        ##########################################
        # add continuous state and input variables
        #! robot_rvar = self.robots_rvar[robot][bezier_segment][dimension,control_point,parallel]
        self.n_parallel = (2*self.world.dim)**2
        self.robots_rvar = [[self.prog.addMVar((self.world.dim,self.robots_ncp[r],self.n_parallel),
                             lb=-self.bigM, ub=self.bigM) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_hvar = [[self.prog.addMVar((1,self.robots_ncp[r],self.n_parallel),
                             lb=self.world.spec.t0, ub=self.world.spec.tf) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]

        # Now obtain the control points of the derivatives as linear combinations of the control points of the original bzs
        self.robots_drvar = [[get_derivative_control_points_parallel_gurobi(self.robots_rvar[r][i],1) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddrvar = [[get_derivative_control_points_parallel_gurobi(self.robots_rvar[r][i],2) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_dhvar = [[get_derivative_control_points_parallel_gurobi(self.robots_hvar[r][i],1) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddhvar = [[get_derivative_control_points_parallel_gurobi(self.robots_hvar[r][i],2) for i in range(self.robots_nbzs[r])] for r in range(self.nrobots)]

        #! object_zonotope = objects_zonotopes[o][bzo][parallel] 
        # object_zonotopes starts from the outer vertices of object_Xf
        self.objects_Xfs = [[[zonotope(x=self.prog.addMVar((2*self.world.dim,),lb=-self.bigM,ub=self.bigM),
                                       Gdiag=self.prog.addMVar((2*self.world.dim,),lb=0,ub=self.bigM)) for _ in range(self.n_parallel)] 
                                                                                                             for _ in range(self.objects_nbzs[o])] 
                                                                                                             for o in range(self.nobjects)]
        self.objects_X0s = [[[zonotope(x=self.prog.addMVar((2*self.world.dim,),lb=-self.bigM,ub=self.bigM),
                                       Gdiag=self.prog.addMVar((2*self.world.dim,),lb=0,ub=self.bigM)) for _ in range(self.n_parallel)]
                                                                                                              for _ in range(self.objects_nbzs[o])] 
                                                                                                              for o in range(self.nobjects)]

        # self.objects_rho = self.prog.addMVar((2,),lb=0,ub=gp.GRB.INFINITY)
        self.objective_rho = self.prog.addVar(lb=0,ub=gp.GRB.INFINITY)
        # self.prog.addConstr(self.objective_rho == gp.max_([rho for rho in self.objects_rho]))
        # now add the robustness metric to the generator of objects_X0s
        for o in range(self.nobjects):
            for bzo in range(self.objects_nbzs[o]):
                for pi in range(self.n_parallel):
                    for d in range(self.world.dim):
                        self.prog.addConstr(self.objects_X0s[o][bzo][pi].Gdiag[d] == 0.0,
                                            name=f"rho_object_{o}_{bzo}_{pi}_Gdiag_{d}")
        
        # object_Xf encompasses the 16 object_zonontopes
        self.objects_Xf = [[zonotope(x=self.prog.addMVar((2*self.world.dim,),lb=-self.bigM,ub=self.bigM),
                                     Gdiag=self.prog.addMVar((2*self.world.dim,),lb=0,ub=self.bigM)) for _ in range(self.objects_nbzs[o])] 
                                                                                                           for o in range(len(self.objects))]
        self.objects_X0 = [[zonotope(x=self.prog.addMVar((2*self.world.dim,),lb=-self.bigM,ub=self.bigM),
                                     Gdiag=self.prog.addMVar((2*self.world.dim,),lb=0,ub=self.bigM)) for _ in range(self.objects_nbzs[o])]
                                                                                                            for o in range(len(self.objects))]
        for o in range(self.nobjects):
            for bzo in range(self.objects_nbzs[o]):
                createSupZonotope(self.prog,self.objects_Xfs[o][bzo],self.objects_Xf[o][bzo])  
                createSupZonotope(self.prog,self.objects_X0s[o][bzo],self.objects_X0[o][bzo])

        #! Constraints
        self._continuity_constraints()
        self._initial_final_position_constraints()
        self._initial_final_velocity_constraints()
        self._initial_final_time_constraints()
        self._positive_time_derivative_constraint()
        self._velocity_constraints()
        self._world_box_constraints()

        # constraints related to forward reachability of objects: zonotopes
        self._object_dynamics()
        self._zonotope_propagation_constraints()
        self._obstacle_collision_constraints()
        # self._object_collision_constraints()

        # embed STL constraints
        self._stl_constraints()

        # add the cost (can be quadratic! MILP or MIQP)
        self.cost_expression = 0*self.robots_rvar[0][0][0,0,0]
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
        print("--- Adding cost function ---")
        weight_L = 0
        weight_V = 0
        weight_A = 0
        weight_absA = 0
        weight_rho = 0

        # # obstacle avoidance
        # weight_absA = 0.001
        # weight_rho = 1000000

        # corridor 
        weight_A = 0.000001
        weight_rho = 1000000000

        # # complex stl
        # weight_A = 0.000001
        # weight_rho = 10000000

        # # pong
        # # weight_A = 0.0001
        # weight_absA = 0.0001
        # weight_rho = 100000

        ### path length cost
        if weight_L > 0:
            try:
                for r in range(self.nrobots):
                    for bz in range(self.robots_nbzs[r]):
                        for pi in range(self.n_parallel):
                            for cp in range(self.robots_ncp[r]-1):
                                for d in range(self.world.dim):
                                    delta = self.prog.addVar(vtype=gp.GRB.CONTINUOUS)
                                    self.cost_expression += delta

                                    self.prog.addConstr(delta*delta >= \
                                                        weight_L*self.robots_drvar[r][bz][d,cp,pi]*self.robots_drvar[r][bz][d,cp,pi])
            except Exception as e:
                print(f"Exception objL: {e}")

        ### path velocity cost
        if weight_V > 0:
            try:
                for r in range(self.nrobots):
                    for bz in range(self.robots_nbzs[r]):
                        for pi in range(self.n_parallel):
                            for cp in range(self.robots_ncp[r]-1):
                                for d in range(self.world.dim):
                                    delta = self.prog.addVar(vtype=gp.GRB.CONTINUOUS)
                                    self.cost_expression += delta

                                    self.prog.addConstr(delta*(self.robots_order[r]*self.robots_dhvar[r][bz][0,cp,pi]) >= \
                                                        weight_V*self.robots_drvar[r][bz][d,cp,pi]*self.robots_drvar[r][bz][d,cp,pi])
            except Exception as e:
                print(f"Exception objV: {e}")

        ### path acceleration cost
        if weight_A > 0:
            try:
                for r in range(self.nrobots):
                    for bz in range(self.robots_nbzs[r]):
                        for pi in range(self.n_parallel):
                            for cp in range(self.robots_ncp[r]-2):
                                for d in range(self.world.dim):
                                    attenuation_term = (2*weight_A)/(1+self.robots_order[r]-2)
                                    self.cost_expression += attenuation_term*self.robots_ddrvar[r][bz][d,cp,pi]*self.robots_ddrvar[r][bz][d,cp,pi]
                                self.cost_expression += attenuation_term*self.robots_ddhvar[r][bz][0,cp,pi]*self.robots_ddhvar[r][bz][0,cp,pi]
            except Exception as e:
                print(f"Exception objU: {e}")

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

                                # ddhvar = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=-self.bigM, ub=self.bigM)
                                # self.prog.addConstr(ddhvar == self.robots_ddhvar[r][bz][0,cp,pi])
                                # abs_ddhvar = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=0, ub=self.bigM)
                                # self.prog.addConstr(abs_ddhvar == gp.abs_(ddhvar))
                                # abs_ddhvars.append(abs_ddhvar)
                cost_var = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=0, ub=self.bigM)
                self.prog.addConstr(cost_var == gp.max_(abs_ddrvars))
                self.cost_expression += weight_absA*cost_var
            except Exception as e:
                print(f"Exception objabsU: {e}")

        if weight_rho > 0:
            try:
                minimize_rho = self.prog.addVar(vtype=gp.GRB.CONTINUOUS, lb=-gp.GRB.INFINITY, ub=0)
                self.prog.addConstr(minimize_rho == -weight_rho*self.objective_rho)
                self.cost_expression += minimize_rho
            except Exception as e:
                print(f"Exception objrho: {e}")

    def _continuity_constraints(self):
        print("--- Adding continuity constraints ---")
        # Bezier curves of the robots should be connected
        # Zonotopes of the objects, X0s, should be connected to the edge points of the previous zonotope, Xf
        for r in range(self.nrobots):
            for pi in range(self.n_parallel):
                for bzr in range(self.robots_nbzs[r]-1):
                    self.prog.addConstrs((self.robots_rvar[r][bzr][d,-1,pi] == self.robots_rvar[r][bzr+1][d,0,pi] for d in range(self.world.dim)),
                                         name=f"robot_{r}_{bzr}_{pi}_space_continuity_constraints")
                    self.prog.addConstr(self.robots_hvar[r][bzr][0,-1,pi] == self.robots_hvar[r][bzr+1][0,0,pi],
                                        name=f"robot_{r}_{bzr}_{pi}_time_continuity_constraints")

        for o in range(self.nobjects):
            for pi in range(self.n_parallel):
                for bzo in range(self.objects_nbzs[o]-1):
                    self.prog.addConstrs((self.objects_X0s[o][bzo+1][pi].x[d] == self.objects_Xf[o][bzo].x[d] + self.signs[pi][d]*self.objects_Xf[o][bzo].Gdiag[d] for d in range(self.world.dim)),
                                         name=f"object_{o}_{bzo}_{pi}_space_continuity_constraints")
             
    def _initial_final_position_constraints(self):
        print("--- Adding initial final position constraints ---")
        # Bezier curves of the robots should start and end at the initial and final positions
        # Zonotopes of the objects should start at the edge points of the initial zonotope, X0d, and end within the final zonotope, Xfd
        for r in range(self.nrobots):
            for pi in range(self.n_parallel):
                try:
                    self.prog.addConstrs((self.robots_rvar[r][0][d,0,pi]==self.robots[r].x0[d] for d in range(self.world.dim)),
                                         name=f"robot_{r}_{pi}_initial_position_constraints")
                    self.prog.addConstrs((self.robots_rvar[r][-1][d,-1,pi]==self.robots[r].xf[d] for d in range(self.world.dim)),
                                         name=f"robot_{r}_{pi}_final_position_constraints")
                except Exception as e:
                    print(f"Error in {r} _initial_final_position_constraints: {e}")

        for o in range(self.nobjects):
            for pi in range(self.n_parallel):
                try:
                    # constrainZinZ(self.prog,self.objects_X0[o][0],self.objects[o].X0d)
                    self.prog.addConstrs((self.objects_X0[o][0].x[d] == self.objects[o].X0d.x[d] for d in range(2*self.world.dim)))
                    self.prog.addConstrs((self.objects_X0[o][0].Gdiag[d] == self.objects[o].X0d.Gdiag[d] for d in range(2*self.world.dim)))
                    constrainZinZ(self.prog,self.objects_Xf[o][-1],self.objects[o].Xfd)
                except Exception as e:
                    print(f"Error in {o} _initial_final_position_constraints: {e}")

    def _initial_final_velocity_constraints(self):
        print("--- Adding initial final velocity constraints ---")
        # Bezier curves of the robots should start and end at the initial and final velocities
        # Zonotopes of the objects should start at the edge points of the initial zonotope, X0d, and end within the final zonotope, Xfd (already done)
        for r in range(self.nrobots):
            for pi in range(self.n_parallel):
                try:
                    self.prog.addConstrs((self.robots_drvar[r][0][i,0,pi]==self.robots_dhvar[r][0][0,:,pi]*self.robots[r].dx0[r] for i in range(self.world.dim)))
                    self.prog.addConstrs((self.robots_drvar[r][-1][i,-1,pi]==self.robots_dhvar[r][-1][0,:,pi]*self.robots[r].dxf[r] for i in range(self.world.dim)))
                except Exception as e:
                    print(f"Error in {r} _initial_final_velocity_constraints: {e}")
        # for the object, this is already being done in the _initial_final_position_constraints (Because it's over the entire zonotope)

    def _initial_final_time_constraints(self):
        print("--- Adding initial final time constraints ---")
        # Bezier curves of the robots should start and end at the initial and final times
        for r in range(self.nrobots):
            for pi in range(self.n_parallel):
                try:
                    self.prog.addConstr(self.robots_hvar[r][0][0,0,pi]==self.world.spec.t0)
                    self.prog.addConstr(self.robots_hvar[r][-1][0,-1,pi]==self.world.spec.tf)
                except Exception as e:
                    print(f"Error in {r} _initial_final_time_constraints: {e}")
    
    def _positive_time_derivative_constraint(self):
        print("--- Adding positive time derivative constraints ---")
        # Time bezier curves of the robots should be strictly positive and bounded
        # Increasing the dh_lb tends to speed up the optimization
        for r in range(self.nrobots):
            for pi in range(self.n_parallel):
                for bz in range(self.robots_nbzs[r]):
                    for cp in range(self.robots_ncp[r]-1):
                        try:
                            self.prog.addConstr(self.robots_dhvar[r][bz][0,cp,pi]>=self.dh_lb,
                                                name=f"robot_{r}_{bz}_{cp}_{pi}_positive_time_derivative_constraint")
                            self.prog.addConstr(self.robots_dhvar[r][bz][0,cp,pi]<=self.dh_ub,
                                                name=f"robot_{r}_{bz}_{cp}_{pi}_positive_time_derivative_constraint")
                        except Exception as e:
                            print(f"Error in {r} {bz} {cp} _positive_time_derivative_constraint: {e}")

    def _velocity_constraints(self):
        print("--- Adding velocity constraints ---")
        # Velocity of the robots should be bounded
        # Velocity of the objects should be bounded, evaluated at the post-impact zonotope center
        for r in range(self.nrobots):
            for pi in range(self.n_parallel):
                for bz in range(self.robots_nbzs[r]):
                    for cp in range(self.robots_ncp[r]-1):
                        try:
                            self.prog.addConstrs((self.robots_drvar[r][bz][i,cp,pi]<=self.robots_dhvar[r][bz][0,cp,pi]*self.robots[r].dq_ub[i] for i in range(self.world.dim)),
                                                 name=f"velocity_constraints_robot_{r}_{bz}_{cp}_{pi}")
                            self.prog.addConstrs((self.robots_drvar[r][bz][i,cp,pi]>=self.robots_dhvar[r][bz][0,cp,pi]*self.robots[r].dq_lb[i] for i in range(self.world.dim)),
                                                 name=f"velocity_constraints_robot_{r}_{bz}_{cp}_{pi}")
                        except Exception as e:
                            print(f"Error in {r} {bz} {cp} _velocity_constraints: {e}")

        for o in range(self.nobjects):
            for pi in range(self.n_parallel):
                for bz in range(self.objects_nbzs[o]):
                    try:
                        self.prog.addConstrs((self.objects_X0s[o][bz][pi].x[d+2]<=self.objects[o].dq_ub[d] for d in range(self.world.dim)),
                                             name=f"velocity_constraints_object_{o}_{bz}_{pi}_{pi}")
                        self.prog.addConstrs((self.objects_X0s[o][bz][pi].x[d+2]>=self.objects[o].dq_lb[d] for d in range(self.world.dim)),
                                             name=f"velocity_constraints_object_{o}_{bz}_{pi}_{pi}")
                    except Exception as e:
                        print(f"Error in {o} {bz} {cp} _velocity_constraints: {e}")

    def _world_box_constraints(self):
        print("--- Adding world box constraints ---")
        # Position of the robots should be within the world box
        # Position of the objects should be within the world box, evaluated at the pre-impact zonotopes for all parallel ones
        for r in range(self.nrobots):
            for pi in range(self.n_parallel):
                for bz in range(self.robots_nbzs[r]):
                    for cp in range(self.robots_ncp[r]):
                        try:
                            self.prog.addConstrs((self.robots_rvar[r][bz][i,cp,pi]<=self.world.x_ub[i] for i in range(self.world.dim)),
                                                 name=f"world_box_constraints_robot_{r}_{bz}_{cp}_{pi}")
                            self.prog.addConstrs((self.robots_rvar[r][bz][i,cp,pi]>=self.world.x_lb[i] for i in range(self.world.dim)),
                                                 name=f"world_box_constraints_robot_{r}_{bz}_{cp}_{pi}")
                        except Exception as e:
                            print(f"Error in {r} {bz} {cp} _world_box_constraints: {e}")

        for o in range(self.nobjects):
            for pi in range(self.n_parallel):
                for bz in range(self.objects_nbzs[o]):
                    for cp in range(self.objects_ncp[o]):
                        try:
                            self.prog.addConstrs((self.objects_Xfs[o][bz][pi].x[d]<=self.world.x_ub[d] for d in range(self.world.dim)),
                                                 name=f"world_box_constraints_object_{o}_{bz}_{cp}")
                            self.prog.addConstrs((self.objects_Xfs[o][bz][pi].x[d]>=self.world.x_lb[d] for d in range(self.world.dim)),
                                                 name=f"world_box_constraints_object_{o}_{bz}_{cp}")
                        except Exception as e:
                            print(f"Error in {o} {bz} {cp} _world_box_constraints: {e}")
    
    def _zonotope_propagation_constraints(self):
        print("--- Adding zonotope propagation constraints ---")
        # loop through all vertices of the object_Xf, add the robustness parameter as a variable in the generator around that vertex
        # and add the forward propogation bilinear constraints (A+IT)x0
        for o in range(self.nobjects):
            for bzo in range(self.objects_nbzs[o]):
                for pi in range(self.n_parallel):
                    # we should only add these zonotope propagation constraints if the object has an impact, otherwise we would keep on propagating, 
                    # we take care of that later in _object_dynamics as there we construct the zs variables
                    # Constraint: objects_Xfs[o][bzo][pi] = (I + A*T)@ (objects_drvar[o][bzo][:,0,pi] \oplus robustness)
                    #         --> objects_Xfs[o][bzo][pi] = (I + A*T)@ objects_X0s[o][bzo][pi]
                    #
                    # (I+AT) = [[1,0,T,0],
                    #           [0,1,0,T],
                    #           [0,0,1,0],
                    #           [0,0,0,1]], because it's a double integrator, this is equal to e^{AT}
                    
                    # first we do the center of the zonotope
                    try:
                        for d in range(self.world.dim):
                            # x_f = x_0 + T*dx_0, position changes because of velocity
                            self.prog.addConstr(self.objects_Xfs[o][bzo][pi].x[d] == self.objects_X0s[o][bzo][pi].x[d] + self.objects[o].dt*self.objects_X0s[o][bzo][pi].x[d+2],
                                                name=f"zonotope_propagation_{o}_{bzo}_{pi}_{d}")
                        for d in range(self.world.dim,2*self.world.dim):
                            # dx_f = dx_0, velocity unchanced because free-floating
                            self.prog.addConstr(self.objects_Xfs[o][bzo][pi].x[d] == self.objects_X0s[o][bzo][pi].x[d],
                                                name=f"zonotope_propagation_{o}_{bzo}_{pi}_{d}")
                    except Exception as e:
                        print(f"Error in {o} {bzo} {pi} _zonotope_propagation_constraints center bilinear: {e}")

                    # now we do the generator of the zonotope, this is easy as long as the generator is always a diagonal matrix
                    try:
                        for d in range(self.world.dim):
                            # This should also consider that Gdiag[d+2] == rho if object.zs_all_bzs[bzo-1] == 1
                            # e.g. if the object had an impact with a robot, then the post-impact velocity should be the robustness parameter
                            # self.prog.addConstr(self.objects_X0s[o][bzo][pi].Gdiag[d+2] >= self.objective_rho -self.bigM*(1-self.objects[o].zs_all_bzs[bzo-1]),
                            #                     name=f"zonotope_propagation_{o}_{bzo}_{pi}_{d+2}_robustness")
                            self.prog.addConstr((self.objects[o].zs_all_bzs[bzo-1] == 1) >> (self.objects_X0s[o][bzo][pi].Gdiag[d+2] == self.objective_rho),
                                                name=f"zonotope_propagation_{o}_{bzo}_{pi}_{d+2}_robustness")
                            self.prog.addConstr((self.objects[o].zs_all_bzs[bzo-1] == 0) >> (self.objects_X0s[o][bzo][pi].Gdiag[d+2] == 0),
                                                name=f"zonotope_propagation_{o}_{bzo}_{pi}_{d+2}_robustness")
                            
                            # G_x = G_x + T*G_dx
                            self.prog.addConstr(self.objects_Xfs[o][bzo][pi].Gdiag[d] == self.objects_X0s[o][bzo][pi].Gdiag[d] + self.objects[o].dt*self.objects_X0s[o][bzo][pi].Gdiag[d+2],
                                                name=f"zonotope_propagation_{o}_{bzo}_{pi}_{d+2}")
                            
                            # G_dx = G_dx
                            self.prog.addConstr(self.objects_Xfs[o][bzo][pi].Gdiag[d+2] == self.objects_X0s[o][bzo][pi].Gdiag[d+2],
                                                name=f"zonotope_propagation_{o}_{bzo}_{pi}_{d+2}")
                    except Exception as e:
                        print(f"Error in {o} {bzo} {pi} _zonotope_propagation_constraints generator bilinear: {e}")

    def _object_dynamics(self):
        print("--- Adding object dynamics ---")
        # the object may not be actuated by itself. It can only be pushed by the robots: elastic impact
        # for the object, the dynamics are: continuous velocity OR elastic impact with robot r
        # for the robot, the dynamics are: dynamics of r OR elastic impact with object o
        for r in range(self.nrobots):
            #! zs[r][o][bzr,bzo] == 1 indicates that bzr of robot r has an impact with bzo of object o
            zs = [self.prog.addMVar((self.robots_nbzs[r]-1,self.objects_nbzs[o]-1),vtype=gp.GRB.BINARY) for o in range(self.nobjects)]
            self.robots[r].zs = zs
            #? for each robot bezier, we can only bump with one object bezier
            for bzr in range(self.robots_nbzs[r]-1):
                # zs_sums[o] indicates that this bzr intersects with any segment of the object
                # this should be less than 1 to constrain that a robot's bezier can only bump with one object's segment
                zs_sums = [self.prog.addVar(vtype=gp.GRB.BINARY) for o in range(self.nobjects)]
                for o in range(self.nobjects):
                    self.prog.addConstr(zs_sums[o] == gp.quicksum([zs[o][bzr,bzo] for bzo in range(self.objects_nbzs[o]-1)]),
                                        name=f"robot_{r}_{bzr}_impact_constraint quicksum")
                self.prog.addConstr(gp.quicksum(zs_sums)<=1)
            
            #? for each robot, two consecutive bzs can never both bump
            #? so we constrain that gp.quicksum([zs[o][bzr,:] for o in range(self.nobjects)]) <= 1
            for bzr in range(self.robots_nbzs[r]-2):
                try:
                    self.prog.addConstr(gp.quicksum([zs[o][bzs,bzo] for o in range(self.nobjects) for bzo in range(self.objects_nbzs[o]-1) for bzs in [bzr,bzr+1]]) <= 1,
                                        name=f"robot_{r}_{bzr}_impact_constraint quicksum")
                except Exception as e:
                    print(f"Error in {r} {bzr} [1,0,1,0] constriant: {e}")

            #! if zs[r][o][bzr,bzo] == 1, let's enforce the impact constraints!
            for o in range(self.nobjects):
                for bzr in range(self.robots_nbzs[r]-1):
                    for bzo in range(self.objects_nbzs[o]-1):
                        for pi in range(self.n_parallel):
                            # if impact, then rvar[r][bzr][:,-1] == rvar[o][bzo][:,-1]
                            # and after impact we get
                            # - drvar[r][bzr+1][:,0] = (mr - mo)/(mr + mo)*drvar[r][bzr][:,-1] + 2*mo/(mr + mo)*drvar[o][bzo][:,-1]
                            # - drvar[o][bzo+1][:,0] = 2*mr/(mr + mo)*drvar[r][bzr][:,-1] + (mo - mr)/(mr + mo)*drvar[o][bzo][:,-1]
                            # otherwise, we add velocity continuity constraints

                            # if impact
                            try:
                                # position and time should be matched between robot and object
                                self.prog.addConstrs((self.robots_rvar[r][bzr][d,-1,pi] >= (self.objects_Xf[o][bzo].x[d]+self.signs[pi][d]*self.objects_Xf[o][bzo].Gdiag[d]) - self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim)),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_1")
                                self.prog.addConstrs((self.robots_rvar[r][bzr][d,-1,pi] <= (self.objects_Xf[o][bzo].x[d]+self.signs[pi][d]*self.objects_Xf[o][bzo].Gdiag[d]) + self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim)),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_2")
                                self.prog.addConstr(self.robots_hvar[r][bzr][0,-1,pi] >= self.objects[o].hvar[bzo][0,-1] - self.bigM*(1-zs[o][bzr,bzo]),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_3")
                                self.prog.addConstr(self.robots_hvar[r][bzr][0,-1,pi] <= self.objects[o].hvar[bzo][0,-1] + self.bigM*(1-zs[o][bzr,bzo]),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_4")
                            except Exception as e:
                                print(f"Error in {o} {bzo} {r} {bzr} if impact constraint: {e}")

                            m1 = (self.robots[r].mass - self.objects[o].mass)/(self.robots[r].mass + self.objects[o].mass)
                            m2 = 2*self.objects[o].mass/(self.robots[r].mass + self.objects[o].mass)
                            m3 = 2*self.robots[r].mass/(self.robots[r].mass + self.objects[o].mass)
                            m4 = (self.objects[o].mass - self.robots[r].mass)/(self.robots[r].mass + self.objects[o].mass)
                            try:
                                # parameter of time derivative for the robot
                                # increasing this parameter makes the approach to the object smoother, but also increases the computation time
                                obj_dhvar = 10

                                # impact dynamics
                                self.prog.addConstrs((self.robots_drvar[r][bzr+1][d,0,pi] >= m1*self.robots_drvar[r][bzr][d,-1,pi] + obj_dhvar*m2*(self.objects_Xf[o][bzo].x[d+2]+self.signs[pi][d+2]*self.objects_Xf[o][bzo].Gdiag[d+2]) - self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim)),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_1")
                                self.prog.addConstrs((self.robots_drvar[r][bzr+1][d,0,pi] <= m1*self.robots_drvar[r][bzr][d,-1,pi] + obj_dhvar*m2*(self.objects_Xf[o][bzo].x[d+2]+self.signs[pi][d+2]*self.objects_Xf[o][bzo].Gdiag[d+2]) + self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim)),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_2")
                                self.prog.addConstr(self.robots_dhvar[r][bzr+1][0,0,pi] >= self.robots_dhvar[r][bzr][0,-1,pi] - self.bigM*(1-zs[o][bzr,bzo]),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_3")
                                self.prog.addConstr(self.robots_dhvar[r][bzr+1][0,0,pi] <= self.robots_dhvar[r][bzr][0,-1,pi] + self.bigM*(1-zs[o][bzr,bzo]),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_4")

                                self.prog.addConstrs((obj_dhvar*self.objects_X0s[o][bzo+1][pi].x[d+2] >= m3*self.robots_drvar[r][bzr][d,-1,pi] + obj_dhvar*m4*(self.objects_Xf[o][bzo].x[d+2]+self.signs[pi][d+2]*self.objects_Xf[o][bzo].Gdiag[d+2]) - self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim)),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_5")
                                self.prog.addConstrs((obj_dhvar*self.objects_X0s[o][bzo+1][pi].x[d+2] <= m3*self.robots_drvar[r][bzr][d,-1,pi] + obj_dhvar*m4*(self.objects_Xf[o][bzo].x[d+2]+self.signs[pi][d+2]*self.objects_Xf[o][bzo].Gdiag[d+2]) + self.bigM*(1-zs[o][bzr,bzo]) for d in range(self.world.dim)),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_6")
                                self.prog.addConstr(obj_dhvar >= self.robots_dhvar[r][bzr][0,-1,pi] - self.bigM*(1-zs[o][bzr,bzo]),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_7")
                                self.prog.addConstr(obj_dhvar <= self.robots_dhvar[r][bzr][0,-1,pi] + self.bigM*(1-zs[o][bzr,bzo]),
                                                    name=f"impact_{r}_{bzr}_{o}_{bzo}_8")
                            except Exception as e:
                                print(f"Error in {o} {bzo} {r} {bzr} impact dynamics constraint: {e}")

        # for the object, we enforce continuity, only if zs[r][o][bzr,bzo] == 0 for all r
        #! for each object, if bzo never impacts, envorce continuity
        for o in range(self.nobjects):
            zs_all_bzs = [self.prog.addVar(vtype=gp.GRB.BINARY) for bzo in range(self.objects_nbzs[o]-1)]
            self.objects[o].zs_all_bzs = zs_all_bzs
            for bzo in range(self.objects_nbzs[o]-1):
                try:
                    # # e.g., does this segment of this object intersect with any bezier of any robot?
                    # vs = []
                    # for r in range(self.nrobots):
                    #     for bzr in range(self.robots_nbzs[r]-1):
                    #         vs.append(self.robots[r].zs[o][bzr,bzo])
                    # self.prog.addConstr(zs_all_bzs[bzo] == gp.max_(vs),
                    #                     name=f"zs_{o}_{bzo}_{r}")
                    # WAS:
                    self.prog.addConstr(zs_all_bzs[bzo] == gp.max_([self.robots[r].zs[o][bzr,bzo] for bzr in range(self.robots_nbzs[r]-1) for r in range(self.nrobots)]),
                                        name=f"zs_{o}_{bzo}_{r}")
                except Exception as e:
                    print(f"Error in {o} {bzo} {r} zs[r] == max(robots[r].zs[o][:,bzo]): {e}")

                # if zs_all == 0: continuity: tight constraints
                for pi in range(self.n_parallel):
                    try:
                        self.prog.addConstrs((self.objects_X0s[o][bzo+1][pi].x[d] >= (self.objects_Xf[o][bzo].x[d]+self.signs[pi][d]*self.objects_Xf[o][bzo].Gdiag[d]) - self.bigM*zs_all_bzs[bzo] \
                                              for d in range(2*self.world.dim)),name=f"object_continuity_{o}_{bzo}_1")
                        self.prog.addConstrs((self.objects_X0s[o][bzo+1][pi].x[d] <= (self.objects_Xf[o][bzo].x[d]+self.signs[pi][d]*self.objects_Xf[o][bzo].Gdiag[d]) + self.bigM*zs_all_bzs[bzo] \
                                              for d in range(2*self.world.dim)),name=f"object_continuity_{o}_{bzo}_2")
                    except Exception as e:
                        print(f"Error in {o} {bzo} continuity constraint: {e}")

        # for the robot, we enforce continuity, only if zs[r][o][bzr,bzo] == 0 for all bzo 
        #! for each robot, if bzr never impacts, enforce continuity
        for r in range(self.nrobots):
            # zs_all_bzs[bzr] indicates if bzr of robot r has an impact with any object
            zs_all_bzs = [self.prog.addVar(vtype=gp.GRB.BINARY) for bzr in range(self.robots_nbzs[r]-1)]
            self.robots[r].zs_all_bzs = zs_all_bzs
            for bzr in range(self.robots_nbzs[r]-1):
                try:
                    self.prog.addConstr(zs_all_bzs[bzr] == gp.max_([self.robots[r].zs[o][bzr,bzo] for bzo in range(self.objects_nbzs[o]-1) for o in range(self.nobjects)]))
                except Exception as e:
                    print(f"Error in {r} {bzr} zs_all == quicksum(zs): {e}")

            for bzr in range(self.robots_nbzs[r]-1):
                # if zs_all == 0: continuity: continuously differentiable constraints
                for pi in range(self.n_parallel):
                    try:
                        self.prog.addConstrs((self.robots_drvar[r][bzr+1][d,0,pi] >= self.robots_drvar[r][bzr][d,-1,pi] - self.bigM*zs_all_bzs[bzr] for d in range(self.world.dim)),
                                            name=f"robot_continuity_{r}_{bzr}_1")
                        self.prog.addConstrs((self.robots_drvar[r][bzr+1][d,0,pi] <= self.robots_drvar[r][bzr][d,-1,pi] + self.bigM*zs_all_bzs[bzr] for d in range(self.world.dim)),
                                            name=f"robot_continuity_{r}_{bzr}_2")
                        self.prog.addConstr(self.robots_dhvar[r][bzr+1][0,0,pi] >= self.robots_dhvar[r][bzr][0,-1,pi] - self.bigM*zs_all_bzs[bzr],
                                            name=f"robot_continuity_{r}_{bzr}_3")
                        self.prog.addConstr(self.robots_dhvar[r][bzr+1][0,0,pi] <= self.robots_dhvar[r][bzr][0,-1,pi] + self.bigM*zs_all_bzs[bzr],
                                            name=f"robot_continuity_{r}_{bzr}_4")
                    except Exception as e:
                        print(f"Error in {r} {bzr} continuity constraint: {e}")

            for bzr in range(self.robots_nbzs[r]-1):
                # if zs_all == 1: continuity for the first control point only! all parallel points should intersect and have the same derivatives
                for pi in range(self.n_parallel):
                    try:
                        self.prog.addConstrs((self.robots_rvar[r][bzr][d,0,pi] >= self.robots_rvar[r][bzr][d,0,0] - self.bigM*(1-zs_all_bzs[bzr]) for d in range(self.world.dim)),
                                            name=f"impact_{r}_{bzr}___5")
                        self.prog.addConstrs((self.robots_rvar[r][bzr][d,0,pi] <= self.robots_rvar[r][bzr][d,0,0] + self.bigM*(1-zs_all_bzs[bzr]) for d in range(self.world.dim)),
                                            name=f"impact_{r}_{bzr}___6")
                        self.prog.addConstr(self.robots_hvar[r][bzr][0,0,pi] >= self.robots_hvar[r][bzr][0,0,0] - self.bigM*(1-zs_all_bzs[bzr]),
                                            name=f"impact_{r}_{bzr}___7")
                        self.prog.addConstr(self.robots_hvar[r][bzr][0,0,pi] <= self.robots_hvar[r][bzr][0,0,0] + self.bigM*(1-zs_all_bzs[bzr]),
                                            name=f"impact_{r}_{bzr}___8")
                        
                        # also for the velocities
                        self.prog.addConstrs((self.robots_drvar[r][bzr][d,0,pi] >= self.robots_drvar[r][bzr][d,0,0] - self.bigM*(1-zs_all_bzs[bzr]) for d in range(self.world.dim)),
                                            name=f"impact_{r}_{bzr}___9")
                        self.prog.addConstrs((self.robots_drvar[r][bzr][d,0,pi] <= self.robots_drvar[r][bzr][d,0,0] + self.bigM*(1-zs_all_bzs[bzr]) for d in range(self.world.dim)),
                                            name=f"impact_{r}_{bzr}___10")
                        self.prog.addConstr(self.robots_dhvar[r][bzr][0,0,pi] >= self.robots_dhvar[r][bzr][0,0,0] - self.bigM*(1-zs_all_bzs[bzr]),
                                            name=f"impact_{r}_{bzr}___11")
                        self.prog.addConstr(self.robots_dhvar[r][bzr][0,0,pi] <= self.robots_dhvar[r][bzr][0,0,0] + self.bigM*(1-zs_all_bzs[bzr]),
                                            name=f"impact_{r}_{bzr}___12")
                    except Exception as e:
                        print(f"Error in   {r} {bzr} impact bifurcation constraint: {e}")

                # if zs_all == 0: continuity for the last control point only! all parallel points should intersect and have the same derivatives
                for pi in range(self.n_parallel):
                    try:
                        self.prog.addConstrs((self.robots_rvar[r][bzr][d,-1,pi] >= self.robots_rvar[r][bzr][d,-1,0] - self.bigM*(zs_all_bzs[bzr]) for d in range(self.world.dim)),
                                            name=f"impact_{r}_{bzr}___5")
                        self.prog.addConstrs((self.robots_rvar[r][bzr][d,-1,pi] <= self.robots_rvar[r][bzr][d,-1,0] + self.bigM*(zs_all_bzs[bzr]) for d in range(self.world.dim)),
                                            name=f"impact_{r}_{bzr}___6")
                        self.prog.addConstr(self.robots_hvar[r][bzr][0,-1,pi] >= self.robots_hvar[r][bzr][0,-1,0] - self.bigM*(zs_all_bzs[bzr]),
                                            name=f"impact_{r}_{bzr}___7")
                        self.prog.addConstr(self.robots_hvar[r][bzr][0,-1,pi] <= self.robots_hvar[r][bzr][0,-1,0] + self.bigM*(zs_all_bzs[bzr]),
                                            name=f"impact_{r}_{bzr}___8")
                        
                        # also for the velocities
                        self.prog.addConstrs((self.robots_drvar[r][bzr][d,-1,pi] >= self.robots_drvar[r][bzr][d,-1,0] - self.bigM*(zs_all_bzs[bzr]) for d in range(self.world.dim)),
                                            name=f"impact_{r}_{bzr}___9")
                        self.prog.addConstrs((self.robots_drvar[r][bzr][d,-1,pi] <= self.robots_drvar[r][bzr][d,-1,0] + self.bigM*(zs_all_bzs[bzr]) for d in range(self.world.dim)),
                                            name=f"impact_{r}_{bzr}___10")
                        self.prog.addConstr(self.robots_dhvar[r][bzr][0,-1,pi] >= self.robots_dhvar[r][bzr][0,-1,0] - self.bigM*(zs_all_bzs[bzr]),
                                            name=f"impact_{r}_{bzr}___11")
                        self.prog.addConstr(self.robots_dhvar[r][bzr][0,-1,pi] <= self.robots_dhvar[r][bzr][0,-1,0] + self.bigM*(zs_all_bzs[bzr]),
                                            name=f"impact_{r}_{bzr}___12")
                    except Exception as e:
                        print(f"Error in   {r} {bzr} impact bifurcation constraint: {e}")

            # now we just have two situations 
            # 1. if (i-1) == 1 or (i) == 1, we do not enforce equal control points
            # 2. both (i-1) and (i) == 0, we force equal control points
            zs_all_eithers = [self.prog.addVar(vtype=gp.GRB.BINARY) for bzr in range(self.robots_nbzs[r])]
            self.robots[r].zs_all_eithers = zs_all_eithers
            not_zs_all_eithers = [self.prog.addVar(vtype=gp.GRB.BINARY) for bzr in range(self.robots_nbzs[r])]
            self.robots[r].not_zs_all_eithers = not_zs_all_eithers
            self.prog.addConstrs((not_zs_all_eithers[bzr] == 1 - zs_all_eithers[bzr] for bzr in range(self.robots_nbzs[r])))

            # bzr == 0
            self.prog.addConstr(self.robots[r].zs_all_eithers[0] == self.robots[r].zs_all_bzs[0])
            # bzr == -1
            self.prog.addConstr(self.robots[r].zs_all_eithers[-1] == self.robots[r].zs_all_bzs[-1])

            for bzr in range(1,self.robots_nbzs[r]-1):
                try:
                    self.prog.addConstr(self.robots[r].zs_all_eithers[bzr] == gp.max_([self.robots[r].zs_all_bzs[bzr-1],
                                                                                       self.robots[r].zs_all_bzs[bzr]]),
                                        name=f"zs_all_both_{r}_{bzr}")
                except Exception as e:
                    print(f"Error in {r} {bzr} zs_all_both == quicksum(zs): {e}")

            for bzr in range(self.robots_nbzs[r]):
                # if zs_all_either == 1: the duration of the segment should be limited: we have a lower and upper bound
                try:
                    self.prog.addConstr(self.robots_hvar[r][bzr][0,-1,0] - self.robots_hvar[r][bzr][0,0,0] <= 30.0 + self.bigM*(1-self.robots[r].zs_all_eithers[bzr]),
                                        name=f"impact_{r}_{bzr}__5")
                    self.prog.addConstr(self.robots_hvar[r][bzr][0,-1,0] - self.robots_hvar[r][bzr][0,0,0] >= 5.0 - self.bigM*(1-self.robots[r].zs_all_eithers[bzr]),
                                        name=f"impact_{r}_{bzr}__5")
                except Exception as e:
                    print(f"Error in  {r} {bzr} if impact constraint: {e}")

                # if zs_all_either == 0: all control points of the segment should be equal because the curves have to be equal
                for pi in range(1,self.n_parallel):
                    try:
                        for cp in range(self.robots_ncp[r]):
                            self.prog.addConstrs((self.robots_rvar[r][bzr][d,cp,pi] >= self.robots_rvar[r][bzr][d,cp,0] - self.bigM*self.robots[r].zs_all_eithers[bzr] for d in range(self.world.dim)),
                                                name=f"robot_continuity_{r}_{bzr}_5")
                            self.prog.addConstrs((self.robots_rvar[r][bzr][d,cp,pi] <= self.robots_rvar[r][bzr][d,cp,0] + self.bigM*self.robots[r].zs_all_eithers[bzr] for d in range(self.world.dim)),
                                                name=f"robot_continuity_{r}_{bzr}_5")
                            self.prog.addConstr(self.robots_hvar[r][bzr][0,cp,pi] >= self.robots_hvar[r][bzr][0,cp,0] - self.bigM*self.robots[r].zs_all_eithers[bzr],
                                                    name=f"continuity_{r}_{bzr}_{pi}_{cp}")
                            self.prog.addConstr(self.robots_hvar[r][bzr][0,cp,pi] <= self.robots_hvar[r][bzr][0,cp,0] + self.bigM*self.robots[r].zs_all_eithers[bzr],
                                                name=f"continuity_{r}_{bzr}_{pi}_{cp}")
                    except Exception as e:
                        print(f"Error in {r} {bzr} continuity constraints 2: {e}")

    def _obstacle_collision_constraints(self):
        print("--- Adding obstacle collision constraints ---")
        #TODO: TEST THIS FUNCTION
        for obs in self.world.obstacles:
            # for each robot
            for r in range(self.nrobots):
                for bzr in range(self.robots_nbzs[r]):
                    for pi in range(self.n_parallel):
                        constrainRoutH(self.prog,
                                    self.robots_rvar[r][bzr][:,:,pi],
                                    obs.H,obs.b,
                                    bigM=self.bigM,condition=1)
        
            # for each object
            for o in range(self.nobjects):
                for bzo in range(self.objects_nbzs[o]):
                    # Create sup zonotope of X0s and Xfs
                    sup = zonotope(x=self.prog.addMVar((2*self.world.dim,),lb=-self.bigM,ub=self.bigM),
                                   Gdiag=self.prog.addMVar((2*self.world.dim,),lb=0,ub=self.bigM))
                    Zs = [self.objects_X0[o][bzo], self.objects_Xf[o][bzo]]
                    createSupZonotope(self.prog,Zs,sup)

                    constrainZoutH(self.prog,
                                   sup,
                                   obs.H,obs.b,
                                   bigM=self.bigM,condition=1)

    def _stl_constraints(self):
        #TODO: TEST THIS FUNCTION
        try:
            for spec,name in zip(self.world.spec.preds,self.world.spec.names):
                robot_vars = {}
                for idx in range(self.nrobots):
                    if self.robots[idx].name == name:
                        hvar, rvar = self.robots_hvar[idx], self.robots_rvar[idx]
                        nbzs, ncp  = self.robots_nbzs[idx], self.robots_ncp[idx]
                        robot_vars = {'hvar': hvar, 'rvar': rvar, 'nbzs': nbzs, 'ncp': ncp, 'npi': self.n_parallel}
                        break
                for idx in range(self.nobjects):
                    if self.objects[idx].name == name:
                        hvar = self.objects[idx].hvar
                        X0s, Xfs = self.objects_X0[idx], self.objects_Xf[idx]
                        nbzs, ncp  = self.objects_nbzs[idx], self.objects_ncp[idx]
                        robot_vars = {'hvar': hvar, 'X0s': X0s, 'Xfs': Xfs, 'nbzs': nbzs, 'ncp': ncp}
                        break
                quant_parse_operator(self,spec,robot_vars)
                self.prog.addConstr(spec.z == 1)
        except Exception as e:
            print(f"Error in stl_constraints: {e}")


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
        self.obj_value = self.prog.objVal

        print(f"robustness value: {self.objective_rho.X}")
        for r in range(self.nrobots):
            for o in range(self.nobjects):
                print(f"Robot {r} Object {o} zs: \n{self.robots[r].zs[o].X}")

        # for r in range(self.nrobots):
        #     print(f"Robot {r} impacts any object: {[self.robots[r].zs_all_bzs[bzr].X for bzr in range(self.robots_nbzs[r]-1)]}")

        # for r in range(self.nrobots):
        #     print(f"Robot {r} impacts i and i-1:  {[self.robots[r].zs_all_eithers[bzr].X for bzr in range(self.robots_nbzs[r])]}")

        for o in range(self.nobjects):
            print(f"Object {o} impacts any robot: {[self.objects[o].zs_all_bzs[bzo].X for bzo in range(self.objects_nbzs[o]-1)]}")
            
        self.write_ids_and_names()

        self.robots_rsol = [[self.robots_rvar[r][bz].X for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_drsol = [[get_derivative_control_points_parallel_gurobi(self.robots_rsol[r][bz],1) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddrsol = [[get_derivative_control_points_parallel_gurobi(self.robots_rsol[r][bz],2) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_hsol = [[self.robots_hvar[r][bz].X for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_dhsol = [[get_derivative_control_points_parallel_gurobi(self.robots_hsol[r][bz],1) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddhsol = [[get_derivative_control_points_parallel_gurobi(self.robots_hsol[r][bz],2) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
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
        self.robots_dqtraj = [[np.zeros((self.world.dim,self.N_eval,self.n_parallel)) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        self.robots_ddqtraj = [[np.zeros((self.world.dim,self.N_eval,self.n_parallel)) for bz in range(self.robots_nbzs[r])] for r in range(self.nrobots)]
        for r in range(self.nrobots):
            for bz in range(self.robots_nbzs[r]):
                for i in range(self.world.dim):
                    self.robots_dqtraj[r][bz][i,:,:] = self.robots_drtraj[r][bz][i,:,:]/self.robots_dhtraj[r][bz][0,:,:]
                    self.robots_ddqtraj[r][bz][i,:,:] = (self.robots_ddrtraj[r][bz][i,:,:]*self.robots_dhtraj[r][bz][0,:,:] - \
                                                       self.robots_drtraj[r][bz][i,:,:]*self.robots_ddhtraj[r][bz][0,:,:])/(self.robots_dhtraj[r][bz][0,:,:]**2)
        
        from utilities.read_write_plan import plan_to_csv, zonotopes_to_csv
        for r in range(self.nrobots):
            robots_rsol = [np.vstack((np.mean(self.robots_rsol[r][bz],axis=2),np.zeros((1,self.robots_ncp[r])))) for bz in range(self.robots_nbzs[r])]
            robots_hsol = [np.mean(self.robots_hsol[r][bz],axis=2) for bz in range(self.robots_nbzs[r])]
            plan_to_csv(robots_rsol,robots_hsol,self.robots[r].ids,self.robots[r].other_names,
                        self.robots[r].name,
                        scenario_name=self.world.specification,
                        path='impact_stl/planner/plans/')
        for o in range(self.nobjects):
            objects_rsol = [np.vstack((np.array([self.objects_X0[o][bz].x.X[0:2],self.objects_Xf[o][bz].x.X[0:2]]).T,np.zeros((1,self.objects_ncp[o])))) for bz in range(self.objects_nbzs[o])]
            objects_hsol = [np.array([i*self.objects[o].dt, (i+1)*self.objects[o].dt]).reshape((1,-1)) for i in range(self.objects_nbzs[o])]
            print(f"objects_hsol: {objects_hsol}")
            plan_to_csv(objects_rsol,objects_hsol,self.objects[o].ids,self.objects[o].other_names,
                        self.objects[o].name,
                        scenario_name=self.world.specification,
                        path='impact_stl/planner/plans/')
        # also write the zonotopes X0[idx] and Xf[idx] to a csv file because we want to plot them
        for o in range(self.nobjects):
            X0s = [zonotope(x=self.objects_X0[o][bz].x.X.reshape((1,-1)),Gdiag=self.objects_X0[o][bz].Gdiag.X.reshape((1,-1))) for bz in range(self.objects_nbzs[o])]
            Xfs = [zonotope(x=self.objects_Xf[o][bz].x.X.reshape((1,-1)),Gdiag=self.objects_Xf[o][bz].Gdiag.X.reshape((1,-1))) for bz in range(self.objects_nbzs[o])]
            zonotopes_to_csv(X0s,Xfs,self.objects[o].ids,self.objects[o].other_names,
                             self.objects[o].name,
                             scenario_name=self.world.specification,
                             path='impact_stl/planner/plans/')


        # create the trajectories of the robots that shall be printed to the .csv file
    def evaluate_t(self,t):
        idxs_robots_p, s_robots_p = [], []

        for pi in range(self.n_parallel):
            t_array_robots = [np.array([self.robots_hsol[r][bzr][0,0,pi] for bzr in range(self.robots_nbzs[r])]) for r in range(self.nrobots)]

            idxs_robots = [np.where(t_array_robots[r]<=t)[0][-1] for r in range(self.nrobots)]

            errors_robots = []
            for r in range(self.nrobots):
                errors_robots.append(lambda s: value_bezier(self.robots_htraj[r][idxs_robots[r]][:,:,pi],s)[0] - t)

            s_robots = []
            for r in range(self.nrobots):
                s_robots.append(sp.optimize.root_scalar(errors_robots[r],bracket=[0,1]).root)

            idxs_robots_p.append(idxs_robots)
            s_robots_p.append(s_robots)

        return idxs_robots_p, s_robots_p


    def plot(self):
        fig = plt.figure(figsize=(20,10))
        gs = GridSpec(2,4, figure=fig)
        ax1 = fig.add_subplot(gs[0:2,0:2])
        ax2 = fig.add_subplot(gs[0,2])
        ax3 = fig.add_subplot(gs[0,3])
        ax4 = fig.add_subplot(gs[1,2])
        ax5 = fig.add_subplot(gs[1,3])

        robot_ls = ['k','k--']
        if True:
            self.world.plot(ax1)
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    ax1.plot(self.robots_rtraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],robot_ls[r])
                    ax1.plot(self.robots_rtraj[r][bz][0,0],self.robots_rtraj[r][bz][1,0],'ko')
                ax1.plot(self.robots_rtraj[r][-1][0,-1],self.robots_rtraj[r][-1][1,-1],'ko')
            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    for pi in range(self.n_parallel):
                        ax1.plot([self.objects_X0s[o][bz][pi].x.X[0]+self.signs[pi][0]*self.objects_X0s[o][bz][pi].Gdiag.X[0],
                                        self.objects_Xfs[o][bz][pi].x.X[0]+self.signs[pi][0]*self.objects_Xfs[o][bz][pi].Gdiag.X[0]],
                                      [self.objects_X0s[o][bz][pi].x.X[1]+self.signs[pi][1]*self.objects_X0s[o][bz][pi].Gdiag.X[1],
                                        self.objects_Xfs[o][bz][pi].x.X[1]+self.signs[pi][1]*self.objects_Xfs[o][bz][pi].Gdiag.X[1]],'r')
            # ax1.set_title("x-y plane")
            ax1.set_xlabel(r"x position [m]")
            ax1.set_ylabel(r"y position [m]")
            ax1.grid(True)
            ax1.set_aspect('equal', 'box')

        if True:
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    ax2.plot(self.robots_htraj[r][bz][0,:],self.robots_rtraj[r][bz][0,:],robot_ls[r])
                    ax2.plot(self.robots_htraj[r][bz][0,0],self.robots_rtraj[r][bz][0,0],'ko')
                ax2.plot(self.robots_htraj[r][-1][0,-1],self.robots_rtraj[r][-1][0,-1],'ko')
            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    for pi in range(self.n_parallel):
                        ax2.plot([self.objects[o].hvar[bz][0,0], self.objects[o].hvar[bz][0,-1]],
                                      [self.objects_X0s[o][bz][pi].x.X[0]+self.signs[pi][0]*self.objects_X0s[o][bz][pi].Gdiag.X[0],
                                        self.objects_Xfs[o][bz][pi].x.X[0]+self.signs[pi][0]*self.objects_Xfs[o][bz][pi].Gdiag.X[0]],'r')
            # ax2.set_title("x-t plane")
            ax2.set_xlabel(r"Time [s]")
            ax2.set_ylabel(r"x position [m]")
            ax2.grid(True)

        if True:
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    ax3.plot(self.robots_htraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],robot_ls[r])
                    ax3.plot(self.robots_htraj[r][bz][0,0],self.robots_rtraj[r][bz][1,0],'ko')
                ax3.plot(self.robots_htraj[r][-1][0,-1],self.robots_rtraj[r][-1][1,-1],'ko')
            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    for pi in range(self.n_parallel):
                        ax3.plot([self.objects[o].hvar[bz][0,0], self.objects[o].hvar[bz][0,-1]],
                                      [self.objects_X0s[o][bz][pi].x.X[1]+self.signs[pi][1]*self.objects_X0s[o][bz][pi].Gdiag.X[1],
                                        self.objects_Xfs[o][bz][pi].x.X[1]+self.signs[pi][1]*self.objects_Xfs[o][bz][pi].Gdiag.X[1]],'r')
            # ax3.set_title("y-t plane")
            ax3.set_xlabel(r"Time [s]")
            ax3.set_ylabel(r"y position [m]")
            ax3.grid(True)

        # if True:
        #     # plot h_values for robots and objects
        #     for r in range(self.nrobots):
        #         for bz in range(self.robots_nbzs[r]):
        #             axs[1,0].plot(self.robots_htraj[r][bz][0,:],robot_ls[r])
        #             axs[1,0].plot(self.robots_htraj[r][bz][0,0],'ko')
        #         axs[1,0].plot(self.robots_htraj[r][-1][0,-1],'ko')

        #     # axs[1,0].set_title("h-t plane")
        #     axs[1,0].set_xlabel(r"Time [s]")
        #     axs[1,0].set_ylabel(r"Phase [-]")
        #     axs[1,0].grid(True)

        if True:
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    ax4.plot(self.robots_htraj[r][bz][0,:],self.robots_dqtraj[r][bz][0,:],robot_ls[r])
                    ax4.plot(self.robots_htraj[r][bz][0,0],self.robots_dqtraj[r][bz][0,0],'ko')
                ax4.plot(self.robots_htraj[r][-1][0,-1],self.robots_dqtraj[r][-1][0,-1],'ko')

            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    for pi in range(self.n_parallel):
                        ax4.plot([self.objects[o].hvar[bz][0,0], self.objects[o].hvar[bz][0,-1]],
                                      [self.objects_X0s[o][bz][pi].x.X[2]+self.signs[pi][2]*self.objects_X0s[o][bz][pi].Gdiag.X[2],
                                        self.objects_Xfs[o][bz][pi].x.X[2]+self.signs[pi][2]*self.objects_Xfs[o][bz][pi].Gdiag.X[2]],'r')
            # ax4.set_title("dx-t plane")
            ax4.set_xlabel(r"Time [s]")
            ax4.set_ylabel(r"x velocity [m/s]")
            ax4.grid(True)

        if True:
            for r in range(self.nrobots):
                for bz in range(self.robots_nbzs[r]):
                    ax5.plot(self.robots_htraj[r][bz][0,:],self.robots_dqtraj[r][bz][1,:],robot_ls[r]) 
                    ax5.plot(self.robots_htraj[r][bz][0,0],self.robots_dqtraj[r][bz][1,0],'ko')
                ax5.plot(self.robots_htraj[r][-1][0,-1],self.robots_dqtraj[r][-1][1,-1],'ko')
            for o in range(self.nobjects):
                for bz in range(self.objects_nbzs[o]):
                    for pi in range(self.n_parallel):
                        ax5.plot([self.objects[o].hvar[bz][0,0], self.objects[o].hvar[bz][0,-1]],
                                      [self.objects_X0s[o][bz][pi].x.X[3]+self.signs[pi][3]*self.objects_X0s[o][bz][pi].Gdiag.X[3],
                                        self.objects_Xfs[o][bz][pi].x.X[3]+self.signs[pi][3]*self.objects_Xfs[o][bz][pi].Gdiag.X[3]],'r')
            # ax5.set_title("dy-t plane")
            ax5.set_xlabel(r"Time [s]")
            ax5.set_ylabel(r"y velocity [m/s]")
            ax5.grid(True)

        fig.tight_layout()

        plt.savefig("impact_stl/planner/figures/plot.png", dpi=300)
        plt.savefig("impact_stl/planner/figures/plot.svg")

        
    def animate(self):
        Neval = 250
        self.t_range = np.linspace(self.world.spec.t0,self.world.spec.tf,Neval)

        self.fig_anim = plt.figure(figsize=(10,10))
        self.ax_anim = plt.axes()
        anim = animation.FuncAnimation(self.fig_anim,self._animate_update,
                                        frames=Neval,interval=self.world.spec.tf/Neval*1e6*2)
        anim.save("impact_stl/planner/figures/animation.mp4",writer="ffmpeg", fps=Neval/self.world.spec.tf)

    def _animate_update(self,i):
        t = self.t_range[i]
        idxs_robots, s_robots = self.evaluate_t(t)
        # for now, all the parallel trajectories are the same so we just take one
        idxs_robots = idxs_robots[0]
        s_robots = s_robots[0]

        self.ax_anim.clear()

        self.world.plot(self.ax_anim)

        # ROBOTS
        rcolors = [(0,0,0),(0.5,0.5,0.5)]
        for r in range(self.nrobots):
            for pi in range(self.n_parallel):
                state = [value_bezier(self.robots_rtraj[r][idxs_robots[r]][:,:,pi],s_robots[r])[0],
                         value_bezier(self.robots_rtraj[r][idxs_robots[r]][:,:,pi],s_robots[r])[1]]
                # plot the circle
                circle = plt.Circle((state[0],state[1]),0.2,fill=True,color=rcolors[r])
                self.ax_anim.add_artist(circle)
                self.ax_anim.plot(state[0],state[1],'ko')

            for bz in range(self.robots_nbzs[r]):
                self.ax_anim.plot(self.robots_rtraj[r][bz][0,:],self.robots_rtraj[r][bz][1,:],color=rcolors[r])
        

        # # OBJECTS
        # for o in range(self.nobjects):
        #     print(f"hvar: {self.objects[o].hvar}")
        #     print(f"t: {t}")
        # idxs_objects = [self.objects[o].evaluate_t(t)[0] for o in range(self.nobjects)]
        # s_objects = [self.objects[o].evaluate_t(t)[1] for o in range(self.nobjects)]
        # # for o in range(self.nobjects):
        # #         for bz in range(self.objects_nbzs[o]):
        # #             for pi in range(self.n_parallel):
        # #                 self.ax_anim.plot([self.objects_X0s[o][bz][pi].x.X[0]+self.signs[pi][0]*self.objects_X0s[o][bz][pi].Gdiag.X[0],
        # #                                     self.objects_Xfs[o][bz][pi].x.X[0]+self.signs[pi][0]*self.objects_Xfs[o][bz][pi].Gdiag.X[0]],
        # #                                   [self.objects_X0s[o][bz][pi].x.X[1]+self.signs[pi][1]*self.objects_X0s[o][bz][pi].Gdiag.X[1],
        # #                                     self.objects_Xfs[o][bz][pi].x.X[1]+self.signs[pi][1]*self.objects_Xfs[o][bz][pi].Gdiag.X[1]],'r')
                        
        # for o in range(self.nobjects):
        #     for pi in range(self.n_parallel):
        #         state = (1-s_objects[o])*self.objects_X0s[o][idxs_objects[o]][pi].x.X + (s_objects[o])*self.objects_Xfs[o][idxs_objects[o]][pi].x.X
        #         z = zonotope(x=state,Gdiag=(1-s_objects[o])*self.objects_X0s[o][idxs_objects[o]][pi].Gdiag.X + (s_objects[o])*self.objects_Xfs[o][idxs_objects[o]][pi].Gdiag.X)
        #         z.plot(self.ax_anim)
        #         # create a circle of radius 0.2 around the object
        #         circle = plt.Circle((state[0],state[1]),0.2,fill=True,color='r')
        #         self.ax_anim.add_artist(circle)
        #         # self.ax_anim.plot(state[0],state[1],'ro')
    
        self.ax_anim.set_title("x-y plane")
        self.ax_anim.set_xlabel("x [m]")
        self.ax_anim.set_ylabel("y [m]")
        self.ax_anim.set_aspect("equal")