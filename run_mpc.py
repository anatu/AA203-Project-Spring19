# run_mpc.py
import numpy as np
import scipy as sp
import cvxpy as cp
import pdb
import matplotlib.pyplot as plt

# from mpc import MPC_Controller
# from to_cvx_problem import CVX_Problem

A = np.array([[1,1],[0,1]])
B = np.array([[0],[1]])

n,m = np.shape(B)
P = np.eye(n)
Q = np.eye(n)
R = 10*np.eye(m)

Phalf = sp.linalg.sqrtm(P)
Qhalf = sp.linalg.sqrtm(Q)
Rhalf = np.sqrt(R)

T = 3
T_tot = 10

xbar = 5
ubar = .5

x_max = xbar; x_min = -xbar;
u_max = ubar; u_min = -ubar;

u_opt = np.zeros((m,T_tot))
X_opt = np.zeros((n,T_tot+1))

x0 = np.array([[-4.5],[2]])
x = x0
X_opt[:,[0]] = x0
J_opt = 0
xf = np.array([[0],[0]])


obj = 0.0
for i in range(0,T_tot):
    X = cp.Variable((n,T+1))
    U = cp.Variable((m,T))
    # pdb.set_trace()
    constraints = [   cp.max(X) <= x_max,
                      cp.min(X) >= x_min,
                      cp.max(U) <= u_max,
                      cp.min(U) >= u_min,
                      X[:,[0]] == x0,
#                      X[:,[-1]] == xf
                      X[:,1:] == cp.matmul(A,X[:,:-1])+cp.matmul(B,U)
                ]

    for j in range(T):
        obj += cp.atoms.quad_form(X[:,j], Q)
        obj += cp.atoms.quad_form(U[:,j], R)

    obj += cp.atoms.quad_form(X[:,T], Q)
    


    # pdb.set_trace()
    prob = cp.Problem(cp.Minimize(obj),constraints)
    result = prob.solve()

    if result == float('-inf') or result == float('inf'):
        print('problem Unbounded... ')
        J_opt = float('inf')
    else:
        u = U[:,[0]].value
        u_opt[:,i] = u
        x = np.matmul(A,x) + B*u
        X_opt[:,[i+1]] = x

np.set_printoptions(precision=2)

plt.figure()
plt.plot(X_opt[0,:],X_opt[1,:])
plt.show()
pdb.set_trace()
# prob = generateProblem(A,B,P,Q);

# ctrl = MPC_Controller();
''' MATLAB output

X_opt =
   -4.5000   -2.5000   -1.0000    0.0000    0.5000    0.7051    0.6101    0.3137    0.0974    0.0090   -0.0314
    2.0000    1.5000    1.0000    0.5000    0.2051   -0.0950   -0.2964   -0.2163   -0.0884   -0.0404   -0.0186
'''
