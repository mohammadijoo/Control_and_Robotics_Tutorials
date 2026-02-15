from roboticstoolbox import DHRobot, RevoluteDH

# Simple 1-DOF revolute joint with unit-link parameters (toy example)
robot = DHRobot([RevoluteDH(a=0.5, d=0.0, alpha=0.0)], name="one_link")

# Joint inertia about the axis at configuration q = 0
M = robot.inertia([0.0])
J_est = float(M[0, 0])

print("Estimated joint inertia J_est:", J_est)
