
import numpy as np

# Time grid
T_end = 40.0
dt = 0.01
t = np.arange(0.0, T_end, dt)
N = len(t)

# Regressor dimension m = 2
def phi_pe(tt):
    # Rich regressor: different frequencies
    return np.array([np.sin(0.7 * tt), np.cos(1.3 * tt)])

def phi_poor(tt):
    # Nearly collinear components: poor excitation
    s = np.sin(0.7 * tt)
    return np.array([s, 2.0 * s])

# Build samples
Phi_pe = np.zeros((N, 2))
Phi_poor = np.zeros((N, 2))
for k in range(N):
    Phi_pe[k, :] = phi_pe(t[k])
    Phi_poor[k, :] = phi_poor(t[k])

def gramian(Phi, dt):
    # G_N = dt * sum phi(k) phi(k)^T
    G = np.zeros((Phi.shape[1], Phi.shape[1]))
    for k in range(Phi.shape[0]):
        phi_k = Phi[k, :].reshape(-1, 1)
        G += phi_k @ phi_k.T
    return dt * G

G_pe = gramian(Phi_pe, dt)
G_poor = gramian(Phi_poor, dt)

eig_pe = np.linalg.eigvalsh(G_pe)
eig_poor = np.linalg.eigvalsh(G_poor)

print("Eigenvalues (PE regressor)   :", eig_pe)
print("Eigenvalues (poor regressor):", eig_poor)

# Simple scalar identification example:
theta_star = 2.0    # true parameter
gamma = 5.0         # adaptation gain
theta_hat_pe = 0.0
theta_hat_poor = 0.0

for k in range(N - 1):
    # Use only the first component as scalar regressor
    phi1 = Phi_pe[k, 0]
    phi2 = Phi_poor[k, 0]
    y1 = phi1 * theta_star
    y2 = phi2 * theta_star

    e1 = y1 - phi1 * theta_hat_pe
    e2 = y2 - phi2 * theta_hat_poor

    # Forward Euler discretization of theta_dot = -gamma * phi * e
    theta_hat_pe += dt * (-gamma * phi1 * e1)
    theta_hat_poor += dt * (-gamma * phi2 * e2)

print("Estimated theta (PE regressor)   :", theta_hat_pe)
print("Estimated theta (poor regressor):", theta_hat_poor)
