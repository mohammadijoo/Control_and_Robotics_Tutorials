import numpy as np
import matplotlib.pyplot as plt

# Link lengths
l1 = 1.0
l2 = 0.7

# Joint limits (radians)
theta1_min, theta1_max = -np.pi, np.pi
theta2_min, theta2_max = -np.pi, np.pi

# Sampling resolution
n1 = 300
n2 = 300

theta1_vals = np.linspace(theta1_min, theta1_max, n1)
theta2_vals = np.linspace(theta2_min, theta2_max, n2)

xs = []
ys = []
manip_vals = []

for th1 in theta1_vals:
    for th2 in theta2_vals:
        # Forward kinematics (position of the end-effector)
        x = l1 * np.cos(th1) + l2 * np.cos(th1 + th2)
        y = l1 * np.sin(th1) + l2 * np.sin(th1 + th2)
        xs.append(x)
        ys.append(y)

        # Planar position Jacobian J_pos in R2 x R2
        # J = [ -l1 sin(th1) - l2 sin(th1 + th2),  -l2 sin(th1 + th2)
        #        l1 cos(th1) + l2 cos(th1 + th2),   l2 cos(th1 + th2) ]
        j11 = -l1 * np.sin(th1) - l2 * np.sin(th1 + th2)
        j12 = -l2 * np.sin(th1 + th2)
        j21 = l1 * np.cos(th1) + l2 * np.cos(th1 + th2)
        j22 = l2 * np.cos(th1 + th2)
        J = np.array([[j11, j12],
                      [j21, j22]])

        # Determinant as a simple manipulability measure
        manip = np.abs(np.linalg.det(J))
        manip_vals.append(manip)

xs = np.array(xs)
ys = np.array(ys)
manip_vals = np.array(manip_vals)

# Normalize manipulability for color mapping
if np.max(manip_vals) > 0:
    manip_norm = manip_vals / np.max(manip_vals)
else:
    manip_norm = manipul_vals

plt.figure()
sc = plt.scatter(xs, ys, c=manip_norm, s=2)
plt.colorbar(sc, label="Normalized manipulability")
plt.gca().set_aspect("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Planar 2R workspace colored by manipulability")
plt.show()
      
