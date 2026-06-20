import numpy as np

# Simple 2R manipulator parameters
l1 = 1.0
l2 = 1.0

def jacobian_2r(q1, q2):
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)
    J = np.array([
        [-l1 * s1 - l2 * s12, -l2 * s12],
        [ l1 * c1 + l2 * c12,  l2 * c12]
    ], dtype=float)
    return J

def kappa_2(J):
    # 2-norm condition number via singular values
    u, s, vt = np.linalg.svd(J)
    sigma_max = s[0]
    sigma_min = s[-1]
    return sigma_max / sigma_min

# Sample grid of configurations
q1_vals = np.linspace(-np.pi, np.pi, 81)
q2_vals = np.linspace(-np.pi, np.pi, 81)

kappa_map = np.zeros((len(q1_vals), len(q2_vals)))
for i, q1 in enumerate(q1_vals):
    for j, q2 in enumerate(q2_vals):
        J = jacobian_2r(q1, q2)
        # Avoid configurations where J is exactly singular in floating point
        if np.linalg.matrix_rank(J) < 2:
            kappa_map[i, j] = np.inf
        else:
            kappa_map[i, j] = kappa_2(J)

# Example: pick a configuration and simulate sensitivity
q1 = 0.0
q2 = 0.9
J = jacobian_2r(q1, q2)
kappa = kappa_2(J)
print("Condition number at (q1, q2) =", (q1, q2), "is", kappa)

# Joint noise model: small Gaussian perturbation
delta_q = np.array([1.0, -0.8]) * (np.pi / 180.0)  # 1 deg and -0.8 deg
delta_x = J @ delta_q

print("Joint perturbation (rad):", delta_q)
print("Task-space perturbation (m):", delta_x)
print("Amplification factor ||delta_x|| / ||delta_q|| =",
      np.linalg.norm(delta_x) / np.linalg.norm(delta_q))
      
