from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np

# Define a simple 2R planar arm in the xy-plane
l1, l2 = 1.0, 0.7
robot = DHRobot([
    RevoluteDH(a=l1, alpha=0.0, d=0.0),
    RevoluteDH(a=l2, alpha=0.0, d=0.0)
], name="planar2R")

q = np.deg2rad([30.0, 20.0])
dq = np.array([0.5, -0.3])
ddq = np.array([0.2, 0.1])

# Spatial Jacobian at the end-effector
J6 = robot.jacob0(q)  # 6x2: linear and angular velocity
xdot6 = J6 @ dq
# For pure planar motion, use the first two rows of J6 for xy-linear velocity.
      
