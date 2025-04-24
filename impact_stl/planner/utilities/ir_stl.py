import gurobipy as gp
from Spec import Pred, Spec
from utilities.opt_helpers import constrainRinH, constrainRoutH, \
                                  constrainZinH, constrainZoutH, \
                                    createSupZonotope
from utilities.zonotopes import zonotope

def quant_parse_operator(obj,pred,robot_vars:dict):
    print(f"type: {pred.type}")
    try:
        for predi in pred.preds:
            print(f"type in loop: {predi.type}")
            quant_parse_operator(obj,predi,robot_vars)
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
        quant_MU_predicate(obj,pred,robot_vars)

    elif pred.type == "NEG":
        quant_NEG_predicate(obj,pred,robot_vars)

    else:
        ValueError(f"Unknown operator {pred.type}")


def quant_AND(obj,pred,robot_vars:dict):
    print("\n--- parsing AND ---")
    pred.z = obj.prog.addVar(vtype=gp.GRB.BINARY)
    try:
        z_ps = [pred.preds[i].z for i in range(len(pred.preds))]
        obj.prog.addConstr(pred.z == gp.min_(z_ps))
    except Exception as e:
        print(f"Error in AND: {e}")


def quant_OR(obj,pred,robot_vars:dict):
    print("\n--- parsing OR ---")
    pred.z = obj.prog.addVar(vtype=gp.GRB.BINARY)
    try:
        z_ps = [pred.preds[i].z for i in range(len(pred.preds))]
        obj.prog.addConstr(pred.z == gp.max_(z_ps))
    except Exception as e:
        print(f"Error in OR: {e}")


def quant_EVENTUALLY_predicate(obj,pred,robot_vars:dict):
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


def quant_ALWAYS_predicate(obj,pred,robot_vars:dict):
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


def quant_MU_predicate(obj,mu,robot_vars:dict,rho=0):
    print("\n--- parsing MU ---")
    mu.zs = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)

    # check if robot_vars contains the rvar field
    if 'rvar' in robot_vars.keys():
        try:
            print("considering it as robot")
            for idx in range(robot_vars['nbzs']):
                for pi in range(robot_vars['npi']):
                    constrainRinH(obj,
                                robot_vars['rvar'][idx][:,:,pi],
                                mu.preds.H,mu.preds.b,
                                condition=mu.zs[idx])
        except Exception as e:
            print(f"Error in MU: {e}")
    else:
        try:
            print("considering it as an object")
            for idx in range(robot_vars['nbzs']):
                sup = zonotope(x=obj.prog.addMVar((2*2,),lb=-1e5,ub=1e5),
                            Gdiag=obj.prog.addMVar((2*2,),lb=0,ub=1e5))
                Zs = [robot_vars['X0s'][idx], robot_vars['Xfs'][idx]]
                createSupZonotope(obj.prog,Zs,sup)

                constrainZinH(obj.prog,
                                sup,
                                mu.preds.H,mu.preds.b,
                                condition=mu.zs[idx])
        except Exception as e:
            print(f"Error in MU: {e}")


def quant_NEG_predicate(obj,mu,robot_vars:dict,rho=0):
    print("\n--- parsing NEG ---")
    mu.zs = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)
    # check if robot_vars contains the rvar field
    if 'rvar' in robot_vars.keys():
        try:
            print("considering it as robot")
            for idx in range(robot_vars['nbzs']):
                for pi in range(robot_vars['npi']):
                    constrainRoutH(obj,
                                robot_vars['rvar'][idx][:,:,pi],
                                mu.preds.H,mu.preds.b,
                                condition=mu.zs[idx])
        except Exception as e:
            print(f"Error in MU: {e}")
    else:
        try:
            print("considering it as an object")
            for idx in range(robot_vars['nbzs']):
                sup = zonotope(x=obj.prog.addMVar((2*2,),lb=-1e5,ub=1e5),
                            Gdiag=obj.prog.addMVar((2*2,),lb=0,ub=1e5))
                Zs = [robot_vars['X0s'][idx], robot_vars['Xfs'][idx]]
                createSupZonotope(obj.prog,Zs,sup)

                constrainZoutH(obj.prog,
                                sup,
                                mu.preds.H,mu.preds.b,
                                condition=mu.zs[idx])
        except Exception as e:
            print(f"Error in MU: {e}")



def parse_time_any(obj,pred,I,robot_vars):
    # return an array of binary variables which indicate for each bezier segment
    # whether or not it is intersecting interval I at any point along hvar
    z_time = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)

    for idx in range(robot_vars['nbzs']):

        try:
            if 'rvar' in robot_vars.keys():
                b1 = obj.prog.addVar(vtype=gp.GRB.BINARY)
                b2 = obj.prog.addVar(vtype=gp.GRB.BINARY)

                # if obj.h_var[0,-1,idx] >= pred.range[0] then b1 = 1
                print(f"hvar: {robot_vars['hvar'][idx]}, I: {I}")
                obj.prog.addConstr(robot_vars['hvar'][idx][0,-1] >= I[0] - obj.bigM*(1-b1),
                                    name=f"conditional constraint {pred.get_string()} {idx} 1")
                obj.prog.addConstr(robot_vars['hvar'][idx][0,-1] <= I[0] + obj.bigM*b1,
                                    name=f"conditional constraint {pred.get_string()} {idx} 1")
                
                # if obj.h_var[0,0,idx] <= pred.I[1] then b2 = 1
                obj.prog.addConstr(robot_vars['hvar'][idx][0,0] <= I[1] + obj.bigM*(1-b2),
                                    name=f"conditional constraint {pred.get_string()} {idx} 2")
                obj.prog.addConstr(robot_vars['hvar'][idx][0,0] >= I[1] - obj.bigM*b2,
                                    name=f"conditional constraint {pred.get_string()} {idx} 2")

                # now if z_time[idx] == 1, the first and last control points are inside the time range
                obj.prog.addConstr(z_time[idx] == gp.min_([b1,b2]), #c,
                                    name=f"parse_time z_time = c {pred.get_string()} {idx}")
            else:
                print(f"hvar: {robot_vars['hvar'][idx]}, I: {I}")
                if robot_vars['hvar'][idx][0,-1] >= I[0] and robot_vars['hvar'][idx][0,0] <= I[1]:
                    obj.prog.addConstr(z_time[idx] == 1)
                else:
                    obj.prog.addConstr(z_time[idx] == 0)

        except Exception as e:
            print(f"Error in parse_time: {e}")
    
    return z_time
            

    #     mu.zs = obj.prog.addMVar(shape=(robot_vars['nbzs'],),vtype=gp.GRB.BINARY)
    # try:
    #     for idx in range(robot_vars['nbzs']):
    #         obj.prog.addConstr(mu.zs[idx] == 1-mu.preds[0].zs[idx])
    # except Exception as e:
    #     print(f"Error in NEG: {e}")