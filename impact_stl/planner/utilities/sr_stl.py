import gurobipy as gp
from Spec import Pred, Spec

def quant_parse_operator(obj,pred,robot_vars,rho):
    print(f"type: {pred.type}")
    try:
        for predi in pred.preds:
            print(f"type in loop: {predi.type}")
            quant_parse_operator(obj,predi,robot_vars,rho)
    except:
        pass
        print("reached the end")

    if pred.type == "AND":
        quant_AND(obj,pred,robot_vars)

    elif pred.type == "OR":
        quant_OR(obj,pred,robot_vars)

    elif pred.type == "F":
        quant_EVENTUALLY_predicate(obj,pred,robot_vars)
    
    elif pred.type == "G":
        quant_ALWAYS_predicate(obj,pred,robot_vars)

    elif pred.type == "MU":
        quant_MU_predicate(obj,pred,robot_vars,rho)

    elif pred.type == "NEG":
        quant_NEG_predicate(obj,pred,robot_vars,rho)

    else:
        ValueError(f"Unknown operator {pred.type}")


def quant_AND(obj,pred,robot_vars):
    print("\n--- parsing AND ---")
    pred.z = obj.prog.addVar(vtype=gp.GRB.BINARY)
    try:
        z_ps = [pred.preds[i].z for i in range(len(pred.preds))]
        obj.prog.addConstr(pred.z == gp.min_(z_ps))
    except Exception as e:
        print(f"Error in AND: {e}")


def quant_OR(obj,pred,robot_vars):
    print("\n--- parsing OR ---")
    pred.z = obj.prog.addVar(vtype=gp.GRB.BINARY)
    try:
        z_ps = [pred.preds[i].z for i in range(len(pred.preds))]
        obj.prog.addConstr(pred.z == gp.max_(z_ps))
    except Exception as e:
        print(f"Error in OR: {e}")


def quant_EVENTUALLY_predicate(obj,pred,robot_vars):
    print("\n--- parsing EVENTUALLY ---")
    pred.z_time = parse_time_any(obj,pred,pred.I,robot_vars)
    pred.zs = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)
    pred.z = obj.prog.addVar(vtype=gp.GRB.BINARY)
    try:
        for idx in range(robot_vars['nbzs']):
            obj.prog.addConstr(pred.zs[idx] == gp.min_([pred.z_time[idx],pred.preds[0].zs[idx]]))
        obj.prog.addConstr(pred.z == gp.max_([z for z in pred.zs]))
    except Exception as e:
        print(f"Error in EVENTUALLY: {e}")


def quant_ALWAYS_predicate(obj,pred,robot_vars):
    print("\n--- parsing ALWAYS ---")
    pred.z_time = parse_time_any(obj,pred,pred.I,robot_vars)
    pred.zs = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)
    pred.z = obj.prog.addVar(vtype=gp.GRB.BINARY)
    try:
        for idx in range(robot_vars['nbzs']):
            obj.prog.addConstr(pred.zs[idx] == gp.min_([pred.z_time[idx],pred.preds[0].zs[idx]]))
        obj.prog.addConstr(pred.z == gp.min_([z for z in pred.zs]))
    except Exception as e:
        print(f"Error in ALWAYS: {e}")


def quant_MU_predicate(obj,mu,robot_vars,rho):
    # print("\n--- parsing MU ---")
    # mu.zs = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)
    # try:
    #     # bloat the rho values with bigM
    #     for idx in range(robot_vars['nbzs']):
    #         for face in range(mu.preds.nfaces):
    #             # for each control point and each parallel bezier
    #             for cp in range(robot_vars['ncp']):
    #                 ineqs = mu.preds.H @ robot_vars['rvar'][idx][:,cp]
    #                 c = -ineqs[face] + mu.preds.b[face] - rho
    #                 #! c should be greater than 0
    #                 obj.prog.addConstr(c >= -obj.bigM*(1-mu.zs[idx]))

    # except Exception as e:
    #     print(f"Error in MU: {e}")

    print("\n--- parsing MU ---")
    mu.zs = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)
    try:
        for idx in range(robot_vars['nbzs']):
            # a zs per control point
            cps = [-1]
            zss = obj.prog.addMVar(shape=(len(cps),),vtype=gp.GRB.BINARY)
            for cp in cps:
                for face in range(mu.preds.nfaces):
                    # for each control point and each parallel bezier
                    ineqs = mu.preds.H @ robot_vars['rvar'][idx][:,cp]
                    c = -ineqs[face] + mu.preds.b[face] - rho
                    #! c should be greater than 0
                    obj.prog.addConstr(c >= -obj.bigM*(1-zss[cp]))

            obj.prog.addConstr(mu.zs[idx] == gp.max_([z for z in zss]))

    except Exception as e:
        print(f"Error in MU: {e}")


def quant_NEG_predicate(obj,mu,robot_vars,rho):
    print("\n--- parsing NEG ---")
    mu.zs = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)
    # instead of using mu, now we wish to explicitly constrain that all the 
    # control points lay outside the same face of the polytope
    try:
        for idx in range(robot_vars['nbzs']):
            zs_faces = obj.prog.addMVar(shape=(mu.preds.nfaces,),vtype=gp.GRB.BINARY)
            for face in range(mu.preds.nfaces):
                # for each control point and each parallel bezier
                for cp in range(robot_vars['ncp']):
                    ineqs = mu.preds.H @ robot_vars['rvar'][idx][:,cp]
                    c = -ineqs[face] + mu.preds.b[face] + rho
                    #! c should be less than 0
                    obj.prog.addConstr(c <= obj.bigM*(1-zs_faces[face]))
            obj.prog.addConstr(mu.zs[idx] == gp.max_([z_face for z_face in zs_faces]))
            
    except Exception as e:
        print(f"Error in NEG: {e}")





def parse_time_any(obj,pred,I,robot_vars):
    # return an array of binary variables which indicate for each bezier segment
    # whether or not it is intersecting interval I at any point along hvar
    z_time = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)

    for idx in range(robot_vars['nbzs']):
        b1 = obj.prog.addVar(vtype=gp.GRB.BINARY)
        b2 = obj.prog.addVar(vtype=gp.GRB.BINARY)

        # if obj.h_var[0,-1,idx] >= pred.range[0] then b1 = 1
        try:
            obj.prog.addConstr(robot_vars['hvar'][idx][0,-1] >= I[0] - obj.bigM*(1-b1),
                                name=f"conditional constraint {pred.get_string()} {idx} 1")
            obj.prog.addConstr(robot_vars['hvar'][idx][0,-1] <= I[0] + obj.bigM*b1,
                                name=f"conditional constraint {pred.get_string()} {idx} 1")
            
            # if obj.h_var[0,0,idx] <= pred.I[1] then b2 = 1
            obj.prog.addConstr(robot_vars['hvar'][idx][0,-1] <= I[1] + obj.bigM*(1-b2),
                                name=f"conditional constraint {pred.get_string()} {idx} 2")
            obj.prog.addConstr(robot_vars['hvar'][idx][0,-1] >= I[1] - obj.bigM*b2,
                                name=f"conditional constraint {pred.get_string()} {idx} 2")

            # now if z_time[idx] == 1, the first and last control points are inside the time range
            obj.prog.addConstr(z_time[idx] == gp.min_([b1,b2]), #c,
                                name=f"parse_time z_time = c {pred.get_string()} {idx}")
        except Exception as e:
            print(f"Error in parse_time: {e}")
    
    return z_time

def parse_time_robot_robot(obj,hvars1,hvars2):
    z_time = obj.prog.addVar(vtype=gp.GRB.BINARY)
    b1 = obj.prog.addVar(vtype=gp.GRB.BINARY)
    b2 = obj.prog.addVar(vtype=gp.GRB.BINARY)
    try:
        obj.prog.addConstr(hvars1[0,-1] >= hvars2[0,0] - obj.bigM*(1-b1))
        obj.prog.addConstr(hvars1[0,-1] <= hvars2[0,0] + obj.bigM*b1)
        obj.prog.addConstr(hvars1[0,0] <= hvars2[0,-1] + obj.bigM*(1-b2))
        obj.prog.addConstr(hvars1[0,0] >= hvars2[0,-1] - obj.bigM*b2)
        obj.prog.addConstr(z_time == gp.min_([b1,b2]))
    except Exception as e:
        print(f"Error in parse_time_robot_robot: {e}")

    return z_time