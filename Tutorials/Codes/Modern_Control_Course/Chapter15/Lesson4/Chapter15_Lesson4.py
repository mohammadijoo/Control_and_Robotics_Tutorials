# Chapter15_Lesson4.py
# Condition Numbers and Sensitivity in State Reconstruction
#
# Required packages:
#   pip install numpy scipy
#
# Optional related libraries for larger Modern Control workflows:
#   python-control: state-space models, observability matrix, gramians
#   slycot: faster Lyapunov/Gramian routines when available

import numpy as np
from scipy.linalg import expm, eigvalsh
from numpy.linalg import solve, cond, norm

def finite_observability_gramian(A, C, T=5.0, N=4000):
    """Compute W_o(T) = integral_0^T exp(A^T t) C^T C exp(A t) dt."""
    n = A.shape[0]
    W = np.zeros((n, n), dtype=float)
    dt = T / N

    for k in range(N):
        # Midpoint quadrature gives better accuracy than left-endpoint quadrature.
        t = (k + 0.5) * dt
        Phi = expm(A * t)
        W += Phi.T @ C.T @ C @ Phi * dt

    return 0.5 * (W + W.T)

def reconstruct_initial_state(A, C, t_grid, y_samples, ridge=0.0):
    """Solve (W + ridge I) x0_hat = b from sampled output data."""
    n = A.shape[0]
    W = np.zeros((n, n), dtype=float)
    b = np.zeros(n, dtype=float)

    for k in range(len(t_grid) - 1):
        t_mid = 0.5 * (t_grid[k] + t_grid[k + 1])
        dt = t_grid[k + 1] - t_grid[k]
        y_mid = 0.5 * (y_samples[k] + y_samples[k + 1])

        Phi = expm(A * t_mid)
        H = C @ Phi
        W += H.T @ H * dt
        b += (H.T @ np.atleast_1d(y_mid)).ravel() * dt

    W = 0.5 * (W + W.T)
    return solve(W + ridge * np.eye(n), b), W, b

def main():
    # A weakly observed second state: epsilon makes the second state visible,
    # but only faintly. The system remains observable for epsilon != 0 when
    # the two modes are distinct, yet the reconstruction can be badly conditioned.
    eps = 0.02
    A = np.array([[-1.0, 0.0],
                  [ 0.0,-2.0]])
    C = np.array([[1.0, eps]])

    T = 5.0
    N = 2500
    t = np.linspace(0.0, T, N + 1)

    x0_true = np.array([1.0, -1.5])

    # Output y(t) = C exp(A t) x0 + measurement noise.
    clean = np.array([(C @ expm(A * ti) @ x0_true).item() for ti in t])
    rng = np.random.default_rng(4)
    noise_std = 1.0e-3
    y_noisy = clean + noise_std * rng.standard_normal(clean.shape)

    x0_hat, W, b = reconstruct_initial_state(A, C, t, y_noisy, ridge=0.0)
    x0_ridge, W_ridge, _ = reconstruct_initial_state(A, C, t, y_noisy, ridge=1.0e-5)

    lam = eigvalsh(W)
    kappa = lam[-1] / lam[0]

    print("Finite-horizon observability Gramian W_o(T):")
    print(W)
    print("\nEigenvalues:", lam)
    print("2-norm condition number:", kappa)
    print("numpy cond(W):", cond(W))

    print("\nTrue x0:                  ", x0_true)
    print("Least-squares x0 estimate:", x0_hat)
    print("Ridge x0 estimate:        ", x0_ridge)

    print("\nRelative LS error:", norm(x0_hat - x0_true) / norm(x0_true))
    print("Relative ridge error:", norm(x0_ridge - x0_true) / norm(x0_true))

    # First-order sensitivity bound:
    # ||delta x|| <= ||W^{-1}|| ||delta b||
    clean_hat, W_clean, b_clean = reconstruct_initial_state(A, C, t, clean, ridge=0.0)
    delta_b = b - b_clean
    delta_x = x0_hat - clean_hat

    bound = norm(solve(W, delta_b))
    print("\nActual perturbation norm ||delta x||:", norm(delta_x))
    print("Bound quantity ||W^{-1} delta b||:   ", bound)

if __name__ == "__main__":
    main()
