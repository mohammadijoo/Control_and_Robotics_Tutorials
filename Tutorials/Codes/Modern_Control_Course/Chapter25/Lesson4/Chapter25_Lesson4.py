"""
Chapter25_Lesson4.py
Impact of Model Uncertainty on State-Feedback Designs

Monte Carlo study for a second-order state-feedback controller under
parametric uncertainty in A and actuator-gain uncertainty in B.

Dependencies:
    pip install numpy scipy matplotlib
"""

import numpy as np
from scipy.signal import place_poles
from scipy.linalg import solve_continuous_lyapunov
import matplotlib.pyplot as plt

np.random.seed(25)

# Nominal plant: x_dot = A x + B u
A = np.array([[0.0, 1.0],
              [-2.0, -0.5]])
B = np.array([[0.0],
              [1.0]])

# Desired nominal closed-loop poles
poles = np.array([-2.0, -3.0])
K = place_poles(A, B, poles).gain_matrix
Acl = A - B @ K

print("Nominal K =", K)
print("Nominal closed-loop eigenvalues =", np.linalg.eigvals(Acl))

# Lyapunov margin around the nominal closed loop.
# Acl.T P + P Acl = -Q, with Q = I.
Q = np.eye(2)
P = solve_continuous_lyapunov(Acl.T, -Q)
margin_bound = np.min(np.linalg.eigvals(Q).real) / (2.0 * np.linalg.norm(P, 2))
print("Sufficient perturbation bound ||DeltaAcl||_2 <", margin_bound)

# Monte Carlo uncertainty model:
# A_true = A + dA, B_true = B * (1 + rho), so Acl_true = A + dA - B_true K
N = 5000
sigma_A = 0.05
rho_max = 0.25
worst_real_part = []
unstable_count = 0
stable_examples = []

for _ in range(N):
    dA = sigma_A * np.random.randn(2, 2)
    rho = np.random.uniform(-rho_max, rho_max)
    A_true = A + dA
    B_true = B * (1.0 + rho)
    Acl_true = A_true - B_true @ K
    eigs = np.linalg.eigvals(Acl_true)
    mr = np.max(eigs.real)
    worst_real_part.append(mr)
    if mr >= 0.0:
        unstable_count += 1
    else:
        stable_examples.append(eigs)

worst_real_part = np.array(worst_real_part)
print("Unstable samples:", unstable_count, "out of", N)
print("Worst observed max real part:", np.max(worst_real_part))
print("95th percentile max real part:", np.percentile(worst_real_part, 95))

plt.figure(figsize=(8, 4.5))
plt.hist(worst_real_part, bins=50)
plt.axvline(0.0, linestyle="--")
plt.xlabel("max real part of closed-loop eigenvalues")
plt.ylabel("sample count")
plt.title("Closed-loop pole movement under model uncertainty")
plt.tight_layout()
plt.show()
