import sympy as sp
import casadi as cs

# get x_r and x_o as variables
x_r = sp.symbols('x_r', real=True)
x_o = sp.symbols('x_o', real=True)
dx_r = sp.symbols('dx_r', real=True)
dx_o = sp.symbols('dx_o', real=True)

X = sp.Matrix([x_r, x_o])
dX = sp.Matrix([dx_r, dx_o])

# get the function h = sumsqr(x_r - x_o) - 1.0
h = (x_r - x_o)**2 - 0.6**2

# get the jacobian of h with respect to x_r
dh_dx_r = sp.diff(h, x_r)
print(f"dh_dx_r: {dh_dx_r}")

# get hdot
hdot = dh_dx_r * dx_r
print(f"hdot: {hdot}")