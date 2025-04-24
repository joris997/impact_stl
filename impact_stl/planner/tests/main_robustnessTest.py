import gurobipy as gp
scs = {v: k for k, v in vars(gp.StatusConstClass).items() if k[0].isupper()}

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import pypolycontain as pp 
import time


# assume we have an equation of the form c_1 = c_2 + x for x \in X
# we want to find the solution in the form c_1 = y*c_2