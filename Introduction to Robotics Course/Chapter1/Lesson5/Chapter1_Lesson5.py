
# Mechanical/perception/planning toolchains often start from NumPy/SciPy
import numpy as np

# Simple "configuration" and Jacobian numeric example
q = np.array([0.2, -0.4, 0.1])          # joint vector
J = np.array([[1.0, 0.2, 0.0],          # toy Jacobian
              [0.0, 1.0, 0.3]])
qd = np.array([0.5, 0.0, -0.2])         # joint velocities
yd = J @ qd                              # task velocity

print("q =", q)
print("yd =", yd)

# Typical robotics libraries you will see later:
# - rclpy (ROS2 Python)
# - opencv-python (vision)
# - pinocchio / roboticstoolbox / drake (modeling)
      