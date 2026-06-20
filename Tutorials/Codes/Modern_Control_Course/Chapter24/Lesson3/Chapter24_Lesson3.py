# Chapter24_Lesson3.py
# State transformations for MIMO pole placement
# Requires: numpy. Optional: scipy/control for alternative pole-placement routines.

import numpy as np

np.set_printoptions(precision=5, suppress=True)

# A two-input system in transformed coordinates z.
# Chain 1: z1_dot = z2, z2_dot = u1
# Chain 2: z3_dot = z4, z4_dot = u2
A_bar = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0]
])
B_bar = np.array([
    [0.0, 0.0],
    [1.0, 0.0],
    [0.0, 0.0],
    [0.0, 1.0]
])

# A nonsingular state transformation x = T z.
T = np.array([
    [1.0, 0.2, 0.0, 0.1],
    [0.1, 1.0, 0.2, 0.0],
    [0.0, 0.1, 1.0, 0.3],
    [0.2, 0.0, 0.1, 1.0]
])
T_inv = np.linalg.inv(T)

# Original-coordinate plant.
A = T @ A_bar @ T_inv
B = T @ B_bar

# Desired poles: chain 1 -> {-2,-3}; chain 2 -> {-4,-5}.
# For a double integrator, s^2 + a1*s + a0 gives feedback [a0, a1].
F = np.array([
    [6.0, 5.0, 0.0, 0.0],
    [0.0, 0.0, 20.0, 9.0]
])

# Map transformed-coordinate feedback u = -F z back to original coordinates.
# Since z = T^{-1}x, u = -F T^{-1}x = -Kx.
K = F @ T_inv
A_cl = A - B @ K
A_bar_cl = A_bar - B_bar @ F

print("A in original coordinates:\n", A)
print("B in original coordinates:\n", B)
print("State feedback gain K = F T^{-1}:\n", K)
print("Closed-loop eigenvalues in original coordinates:", np.linalg.eigvals(A_cl))
print("Closed-loop eigenvalues in transformed coordinates:", np.linalg.eigvals(A_bar_cl))

similarity_error = T_inv @ A_cl @ T - A_bar_cl
print("Similarity verification ||T^{-1}(A-BK)T - (A_bar-B_bar F)||_F =",
      np.linalg.norm(similarity_error, ord="fro"))

# Library note:
# scipy.signal.place_poles(A, B, desired_poles) can compute K directly for many MIMO cases,
# but this script shows why transformations make the design transparent.
