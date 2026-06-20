import numpy as np
from numpy.linalg import inv
import scipy.signal as sig

# Optional (recommended) for control workflows:
# pip install control
import control as ct

# Parameters
m, b, k = 1.0, 0.4, 4.0

A = np.array([[0.0, 1.0],
              [-k/m, -b/m]])
B = np.array([[0.0],
              [1.0/m]])
C = np.array([[1.0, 0.0]])
D = np.array([[0.0]])

# State-space model
sys_ss = ct.ss(A, B, C, D)

# Transfer function derived from state-space
sys_tf = ct.tf(sys_ss)
print("G(s) from state-space:", sys_tf)

# Direct polynomial transfer function: 1 / (m s^2 + b s + k)
num = [1.0]
den = [m, b, k]
sys_tf_poly = ct.tf(num, den)

# Compare frequency response G(jw) by two routes
w = np.logspace(-2, 2, 10)
for wi in w:
    s = 1j * wi
    G_ss = C @ inv(s*np.eye(A.shape[0]) - A) @ B + D
    G_tf = (num[0]) / (m*s*s + b*s + k)
    print(f"w={wi:8.4f} | G_ss={G_ss[0,0]: .6e} | G_tf={G_tf: .6e} | diff={abs(G_ss[0,0]-G_tf):.3e}")

# Step response (forced response, IC=0)
t = np.linspace(0, 20, 2000)
t1, y1 = ct.step_response(sys_ss, T=t)      # state-space
t2, y2 = ct.step_response(sys_tf_poly, T=t) # transfer function (internally realized)
print("Max |y_ss - y_tf| on grid:", np.max(np.abs(y1 - y2)))

# Demonstrate the role of initial conditions (nonzero x0) using forced_response
x0 = np.array([1.0, 0.0])  # initial displacement
u = np.ones_like(t)        # step input
t3, y3, x3 = ct.forced_response(sys_ss, T=t, U=u, X0=x0, return_x=True)
print("y(t) includes natural response due to x0 != 0; transfer function alone does not encode this term.")
