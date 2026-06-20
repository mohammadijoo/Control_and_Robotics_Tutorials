
import numpy as np
from numpy.linalg import eig, norm
from scipy.linalg import solve_continuous_lyapunov as lyap
import control

# Nominal scalar joint model: J * q_dd + Kd * q_d + Kp * q = 0
J_nom = 0.5   # nominal inertia
Kp = 50.0
Kd = 5.0

def A_cl(J):
    return np.array([[0.0, 1.0],
                     [-Kp / J, -Kd / J]])

# Nominal closed-loop matrix and eigenvalues
A = A_cl(J_nom)
evals, _ = eig(A)
print("Nominal eigenvalues:", evals)

# Lyapunov-based robustness bound
Q = np.eye(2)
P = lyap(A.T, Q)  # solves A^T P + P A = -Q

lam_min_Q = np.min(np.linalg.eigvals(Q).real)
P_norm = norm(P, 2)
delta_bar = lam_min_Q / (2.0 * P_norm)
print("Conservative bound on ||Delta A||_2:", delta_bar)

# Monte Carlo: sample inertia uncertainty and test stability numerically
rng = np.random.default_rng(0)
num_samples = 200
unstable_count = 0
worst_alpha = -1e9

for k in range(num_samples):
    # Sample inertia within +-30%
    J = J_nom * (1.0 + 0.3 * (2.0 * rng.random() - 1.0))
    A_k = A_cl(J)
    evals_k, _ = eig(A_k)
    alpha_k = np.max(evals_k.real)
    worst_alpha = max(worst_alpha, alpha_k)
    if alpha_k >= 0.0:
        unstable_count += 1

print("Worst spectral abscissa over samples:", worst_alpha)
print("Number of unstable samples:", unstable_count)

# Input-output robustness: L2 gain of disturbance channel
# Disturbance torque enters acceleration: x_dot = A x + B_d * d
B_d = np.array([[0.0],
                [1.0 / J_nom]])
C = np.eye(2)   # measure states for simplicity
G = control.ss(A, B_d, C, 0.0)
hinf_norm, peak_w = control.hinfnorm(G)
print("H-infinity norm of disturbance-to-state map:", hinf_norm)
print("Frequency at which peak occurs:", peak_w)
