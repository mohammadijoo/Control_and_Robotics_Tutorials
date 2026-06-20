# Chapter14_Lesson5.py
"""
Time-varying observability demo for Chapter 14, Lesson 5.

The example computes the continuous-time observability Gramian

    W_o = integral Phi(t,t0)^T C(t)^T C(t) Phi(t,t0) dt

for an LTV system xdot = A(t)x, y = C(t)x, then reconstructs x(t0)
from noiseless output samples using x0 = W_o^{-1} z.
"""

import numpy as np
from scipy.integrate import solve_ivp


def A(t: float) -> np.ndarray:
    """Time-varying state matrix."""
    return np.array(
        [
            [0.0, 1.0],
            [-(2.0 + 0.5 * np.sin(1.3 * t)), -(0.15 + 0.05 * np.cos(t))],
        ],
        dtype=float,
    )


def C(t: float) -> np.ndarray:
    """Time-varying output matrix."""
    return np.array([[1.0, 0.2 * np.sin(0.7 * t)]], dtype=float)


def transition_rhs(t: float, phi_flat: np.ndarray) -> np.ndarray:
    """ODE for Phi(t,t0), flattened column-major."""
    n = 2
    phi = phi_flat.reshape((n, n))
    dphi = A(t) @ phi
    return dphi.reshape(-1)


def compute_phi_grid(t0: float, tf: float, samples: int = 1001):
    n = 2
    t_grid = np.linspace(t0, tf, samples)
    phi0 = np.eye(n).reshape(-1)
    sol = solve_ivp(
        transition_rhs,
        (t0, tf),
        phi0,
        t_eval=t_grid,
        rtol=1e-10,
        atol=1e-12,
    )
    if not sol.success:
        raise RuntimeError(sol.message)
    phis = sol.y.T.reshape((-1, n, n))
    return t_grid, phis


def trapezoid_integral(t_grid: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Integrate a vector or matrix-valued array along t using trapezoids."""
    total = np.zeros_like(values[0])
    for k in range(len(t_grid) - 1):
        dt = t_grid[k + 1] - t_grid[k]
        total += 0.5 * dt * (values[k] + values[k + 1])
    return total


def observability_gramian(t_grid: np.ndarray, phis: np.ndarray) -> np.ndarray:
    terms = []
    for t, phi in zip(t_grid, phis):
        ct = C(float(t))
        terms.append(phi.T @ ct.T @ ct @ phi)
    return trapezoid_integral(t_grid, np.array(terms))


def reconstruct_initial_state(t_grid: np.ndarray, phis: np.ndarray, x0_true: np.ndarray):
    y_samples = []
    z_terms = []
    for t, phi in zip(t_grid, phis):
        ct = C(float(t))
        y = ct @ phi @ x0_true
        y_samples.append(float(y[0]))
        z_terms.append(phi.T @ ct.T @ y)
    z = trapezoid_integral(t_grid, np.array(z_terms)).reshape(-1)
    w = observability_gramian(t_grid, phis)
    x0_hat = np.linalg.solve(w, z)
    return np.array(y_samples), z, w, x0_hat


if __name__ == "__main__":
    t0, tf = 0.0, 6.0
    t_grid, phis = compute_phi_grid(t0, tf)

    w = observability_gramian(t_grid, phis)
    eigvals = np.linalg.eigvalsh(w)

    x0_true = np.array([1.0, -0.7])
    y_samples, z, w, x0_hat = reconstruct_initial_state(t_grid, phis, x0_true)

    print("Observability Gramian W_o:")
    print(w)
    print("\nEigenvalues of W_o:", eigvals)
    print("Condition number:", np.linalg.cond(w))
    print("\nTrue x0:", x0_true)
    print("Reconstructed x0:", x0_hat)
    print("Reconstruction error norm:", np.linalg.norm(x0_hat - x0_true))
