from IR_Impact_STL import IR_Impact_STL
from SR_Impact_STL import SR_Impact_STL
from World import World
from utilities.beziers import eval_bezier

robustness_type = "spatial" # "spatial" or "impact"
specification   = "pingpong" # see respective specifications for the options

if __name__ == "__main__":

    world = World(robustness_type=robustness_type, specification=specification)

    if robustness_type == "spatial":
        opt = SR_Impact_STL(world=world)
    elif robustness_type == "impact":
        opt = IR_Impact_STL(world=world)
    else:
        raise ValueError("robustness_type must be either 'spatial' or 'impact'")
    opt.construct()
    opt.solve()
    opt.evaluate()
    
    opt.plot()
    # opt.animate()
    