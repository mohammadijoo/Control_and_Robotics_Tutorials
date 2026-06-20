
import numpy as np
import matplotlib.pyplot as plt

# Link lengths
l1, l2 = 1.0, 0.7

# Joint limits (radians)
q1_min, q1_max = -np.pi, np.pi
q2_min, q2_max = -np.pi, np.pi

N = 50000
q1 = np.random.uniform(q1_min, q1_max, N)
q2 = np.random.uniform(q2_min, q2_max, N)

# Geometric position (planar)
x = l1*np.cos(q1) + l2*np.cos(q1 + q2)
y = l1*np.sin(q1) + l2*np.sin(q1 + q2)

plt.figure()
plt.scatter(x, y, s=1)
plt.axis("equal")
plt.title("Estimated workspace of planar 2R arm")
plt.xlabel("x")
plt.ylabel("y")
plt.show()
      