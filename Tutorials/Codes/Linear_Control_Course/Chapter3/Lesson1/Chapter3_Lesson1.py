import numpy as np
import control  # pip install control

# Physical parameters (e.g., a small robot axis)
m = 1.0    # kg
b = 0.4    # N·s/m
k = 2.0    # N/m

# State-space matrices for q and q_dot
A = np.array([[0.0,      1.0],
              [-k/m, -b/m]])
B = np.array([[0.0],
              [1.0/m]])
C = np.array([[1.0, 0.0]])  # measure position
D = np.array([[0.0]])

sys = control.ss(A, B, C, D)

# Time vector and step input
T = np.linspace(0.0, 10.0, 2000)
u = np.ones_like(T)  # unit step force

T, y, x = control.forced_response(sys, T, u)

print("Final position approx:", y[-1])

# Robotics context (symbolic, not simulated here):
# Use roboticstoolbox-python to build a 1-DOF revolute joint and obtain its dynamics.
# import roboticstoolbox as rtb
# link = rtb.RevoluteDH(a=0.5, d=0.0, alpha=0.0, m=m)
# robot = rtb.DHRobot([link])
# robot.dynamics()  # returns symbolic M(q), C(q, q_dot), g(q), etc.
# Around a fixed configuration, these dynamics can be approximated by an LTI model.
