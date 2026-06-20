# Chapter25_Lesson2.py
# Role of transmission zeros in state-feedback design limits.
#
# Required libraries:
#   pip install numpy scipy
#
# This script uses a SISO plant with a right-half-plane transmission zero:
#       G(s) = (1 - s)/(s^2 + 3s + 2)
# and shows that static state feedback can move poles but does not move
# the plant transmission zero.

import numpy as np
from scipy.signal import ss2tf

np.set_printoptions(precision=6, suppress=True)


def siso_zero_pole_data(A, B, C, D):
    """Return numerator, denominator, zeros, and poles for a SISO state model."""
    numerator, denominator = ss2tf(A, B, C, D)
    numerator = np.trim_zeros(numerator[0], "f")
    denominator = np.trim_zeros(denominator, "f")
    zeros = np.roots(numerator) if len(numerator) > 1 else np.array([])
    poles = np.roots(denominator)
    return numerator, denominator, zeros, poles


def rosenbrock_matrix(s, A, B, C, D):
    """Rosenbrock system matrix P(s) = [[sI-A, -B], [C, D]]."""
    n = A.shape[0]
    upper = np.hstack((s * np.eye(n) - A, -B))
    lower = np.hstack((C, D))
    return np.vstack((upper, lower))


def numerical_rank(M, tol=1e-9):
    """Numerical rank from singular values."""
    sigma = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(sigma > tol)), sigma


# Plant realization:
# A = [[0, 1], [-2, -3]], B = [[0], [1]], C = [[1, -1]], D = [[0]]
# gives G(s) = (1 - s)/(s^2 + 3s + 2).
A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0],
              [1.0]])
C = np.array([[1.0, -1.0]])
D = np.array([[0.0]])

num, den, zeros_open, poles_open = siso_zero_pole_data(A, B, C, D)

print("Open-loop numerator coefficients:", num)
print("Open-loop denominator coefficients:", den)
print("Open-loop zeros:", zeros_open)
print("Open-loop poles:", poles_open)

# Place closed-loop poles at -5 and -6 by direct companion-form matching.
# Open denominator: s^2 + 3s + 2
# Desired denominator: (s+5)(s+6) = s^2 + 11s + 30
K = np.array([[30.0 - 2.0, 11.0 - 3.0]])  # [28, 8]
Acl = A - B @ K

num_cl, den_cl, zeros_cl, poles_cl = siso_zero_pole_data(Acl, B, C, D)

print("\nState-feedback gain K:", K)
print("Closed-loop numerator coefficients:", num_cl)
print("Closed-loop denominator coefficients:", den_cl)
print("Closed-loop zeros:", zeros_cl)
print("Closed-loop poles:", poles_cl)

# Rosenbrock rank test at the known zero z = +1.
z = 1.0
Pz = rosenbrock_matrix(z, A, B, C, D)
rank_z, sv_z = numerical_rank(Pz)

PKz = rosenbrock_matrix(z, Acl, B, C, D)
rank_cl_z, sv_cl_z = numerical_rank(PKz)

print("\nRosenbrock matrix at s = +1")
print("Open-loop rank:", rank_z, "singular values:", sv_z)
print("Closed-loop rank:", rank_cl_z, "singular values:", sv_cl_z)

# Test at a nonzero point, e.g., s = 0.
s0 = 0.0
rank_0, sv_0 = numerical_rank(rosenbrock_matrix(s0, A, B, C, D))
print("\nRosenbrock matrix at s = 0")
print("Rank:", rank_0, "singular values:", sv_0)

# Frequency-domain interpolation fact:
# If G(z)=0, then for any finite stabilizing controller based on that input-output channel,
# L(z)=G(z)C(z)=0, hence S(z)=1 and T(z)=0.
print("\nDesign-limit interpretation:")
print("The RHP zero at s=+1 remains after state feedback.")
print("Fast pole placement is possible, but stable tracking cannot cancel this zero.")
print("A stable prefilter cannot remove the inverse-response limitation caused by the RHP zero.")
