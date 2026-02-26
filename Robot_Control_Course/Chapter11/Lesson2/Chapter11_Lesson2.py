
import numpy as np
from scipy.linalg import expm
from scipy.signal import place_poles

# 2-DOF approximate joint model: each joint as damped double integrator
n_joints = 2
d = 1.0   # damping coefficient (simplified)
b = 5.0   # torque-to-accel gain (simplified)

# Continuous-time A,B,C for stacked [q1, qdot1, q2, qdot2]^T
Ac = np.array([[0, 1, 0, 0],
               [0, -d, 0, 0],
               [0, 0, 0, 1],
               [0, 0, 0, -d]])
Bc = np.array([[0, 0],
               [b, 0],
               [0, 0],
               [0, b]])
Cc = np.array([[1, 0, 0, 0],
               [0, 0, 1, 0]])  # measure both joint positions

Ts = 0.001  # 1 kHz control loop

# Discretization (simple matrix exponential + integral for B_d)
Ad = expm(Ac * Ts)

# For B_d under ZOH: integral_0^Ts exp(Ac*s) ds * Bc
# Use block matrix trick:
mat = np.block([[Ac, Bc],
                [np.zeros((2, 4)), np.zeros((2, 2))]])
exp_mat = expm(mat * Ts)
Ad_from_block = exp_mat[:4, :4]
Bd = exp_mat[:4, 4:]

# Check they match reasonably
assert np.allclose(Ad, Ad_from_block, atol=1e-9)
Ad = Ad_from_block

# Design discrete-time observer gain L_d
# First, pick desired eigenvalues (faster than controller).
# For example, if controller poles are near 0.9, choose observer near 0.3.
desired_poles = np.array([0.3, 0.35, 0.4, 0.45])

# Dual system: (Ad.T, Cc.T)
pp = place_poles(Ad.T, Cc.T, desired_poles)
Ld = pp.gain_matrix.T

print("Observer gain L_d:")
print(Ld)

# Simple simulation loop (no process noise) showing convergence
x_true = np.array([[0.1],   # q1
                   [0.0],   # qdot1
                   [-0.2],  # q2
                   [0.0]])  # qdot2
x_hat = np.zeros((4, 1))
u = np.zeros((2, 1))  # zero torque for this test

for k in range(1000):
    # True system evolution (for demo: linear model assumed exact)
    x_true = Ad @ x_true + Bd @ u
    y = Cc @ x_true

    # Observer update
    x_hat = Ad @ x_hat + Bd @ u + Ld @ (y - Cc @ x_hat)

# After enough steps, x_hat should be very close to x_true
print("Final estimation error:", (x_true - x_hat).ravel())
