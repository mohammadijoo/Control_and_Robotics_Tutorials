import numpy as np
from scipy.linalg import expm

g = 9.81
Jtheta = 0.02   # pitch inertia (kg*m^2)
dt = 0.01

# Reduced hover subsystem: [x, xdot, theta, q]
A = np.array([[0, 1, 0, 0],
              [0, 0, g, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])
B = np.array([[0],
              [0],
              [0],
              [1.0/Jtheta]])

# Discretize exactly
Ad = expm(A*dt)
Bd = np.linalg.solve(A, (Ad - np.eye(4))) @ B

# Simple PD on theta for position regulation
Kx = np.array([0.8, 1.2, 6.0, 2.5])  # hand-tuned gains
x = np.zeros((4,1))
x[0,0] = 1.0  # 1 m initial error

traj = []
for k in range(2000):
    u = -Kx @ x
    x = Ad @ x + Bd @ u.reshape(1,1)
    traj.append(x.flatten())

traj = np.array(traj)
print("final state:", traj[-1])
      