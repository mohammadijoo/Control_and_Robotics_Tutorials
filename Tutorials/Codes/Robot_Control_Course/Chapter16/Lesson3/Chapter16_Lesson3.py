
import numpy as np

# Physical and control parameters
J = 0.05       # inertia
b = 0.01       # viscous damping
k_p = 50.0
k_d = 2.0

Ts = 0.001     # sampling period (s)
T_final = 5.0
N = int(T_final / Ts)

# Discrete-time matrices (forward Euler approximation)
A_d = np.array([
    [1.0, Ts],
    [-Ts * k_p / J, 1.0 - Ts * (k_d + b) / J]
])
B_d = np.zeros((2, 1))  # no explicit input when expressed in error coordinates

# Reference trajectory (constant)
q_ref = 1.0  # rad
e = np.array([0.0, 0.0])  # [position_error, velocity_error]

# Logging buffers
log_t = np.zeros(N)
log_e = np.zeros((N, 2))
log_u = np.zeros(N)

# Torque saturation
u_min, u_max = -5.0, 5.0

for k in range(N):
    t_k = k * Ts

    # Here q_meas, dq_meas would come from sensors (or a simulator)
    # For this simplified example, we reconstruct from error state:
    q_meas = q_ref - e[0]
    dq_meas = -e[1]

    # PD control law in joint coordinates
    u = -k_p * (q_meas - q_ref) - k_d * dq_meas

    # Saturation and logging
    u = float(np.clip(u, u_min, u_max))

    log_t[k] = t_k
    log_e[k, :] = e
    log_u[k] = u

    # Update error state using the derived matrix A_d
    e = A_d @ e

# Post-run: approximate discrete-time eigenvalues and check stability
eigvals = np.linalg.eigvals(A_d)
spectral_radius = max(abs(eigvals))

print("A_d:")
print(A_d)
print("Eigenvalues:", eigvals)
print("Spectral radius:", spectral_radius)

# Example invariant: energy-like quantity V_k = e^T P e
P = np.eye(2)
V = np.einsum("bi,ij,bj->b", log_e, P, log_e)  # V_k sequence
