
import numpy as np
from scipy.linalg import expm
from control import ss, c2d  # python-control package

# Continuous-time joint model
J = 0.01
b = 0.1
A = np.array([[0.0,   1.0],
              [0.0, -b/J]])     # [[0, 1], [0, -10]]
B = np.array([[0.0],
              [1.0/J]])         # [[0], [100]]
C = np.eye(2)
D = np.zeros((2, 1))

Ts = 1e-3  # 1 kHz sampling

# Exact discrete-time matrices via matrix exponential
Phi = expm(A * Ts)
# Use Gamma = A^{-1}(Phi - I) B when A is nonsingular
Gamma = np.linalg.solve(A, (Phi - np.eye(2))) @ B

print("Phi =\n", Phi)
print("Gamma =\n", Gamma)

# Using python-control for a ZOH discretization
sys_c = ss(A, B, C, D)
sys_d = c2d(sys_c, Ts, method='zoh')

print("Phi (control) =\n", sys_d.A)
print("Gamma (control) =\n", sys_d.B)

# Discrete-time step in a control loop (conceptual)
def joint_step(x_k, u_k):
    """
    x_k: 2x1 numpy array [q, dq]
    u_k: scalar torque at step k
    """
    return Phi @ x_k + Gamma * u_k

# In a ROS controller, joint_step would be called at 1 kHz
# using the current torque command u_k and state estimate x_k.
