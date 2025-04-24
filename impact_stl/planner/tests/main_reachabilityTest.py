import pypolycontain as pp
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

A = np.array([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]])
X0_ = pp.zonotope(x=np.array([1,0,1,0]), G=np.array([[0,0,0,0],
                                                    [0,0,0,0],
                                                    [0,0,1,0],
                                                    [0,0,0,0]]))

T = 10
expAT = sp.linalg.expm(A*T)
taylor_expAT = np.eye(4) + A*T #+ np.linalg.matrix_power(A,2)*T**2/2 + np.linalg.matrix_power(A,3)*T**3/6 + np.linalg.matrix_power(A,4)*T**4/24

Xf_exp_ = pp.zonotope(x=np.dot(expAT,X0_.x), G=np.dot(expAT,X0_.G))
Xf_taylor_ = pp.zonotope(x=np.dot(taylor_expAT,X0_.x), G=np.dot(taylor_expAT,X0_.G))

P = np.array([[1,0,0,0],[0,1,0,0]])
X0 = pp.zonotope(x=np.dot(P,X0_.x), G=np.dot(P,X0_.G))
Xf_exp = pp.zonotope(x=np.dot(P,Xf_exp_.x), G=np.dot(P,Xf_exp_.G))
Xf_taylor = pp.zonotope(x=np.dot(P,Xf_taylor_.x), G=np.dot(P,Xf_taylor_.G))

fig, ax = plt.subplots()
try:
    pp.visualize([X0,Xf_exp,Xf_taylor],fig=fig, ax=ax, alpha=0.25)
except:
    pass
ax.set_aspect('equal', 'box')
plt.show()
