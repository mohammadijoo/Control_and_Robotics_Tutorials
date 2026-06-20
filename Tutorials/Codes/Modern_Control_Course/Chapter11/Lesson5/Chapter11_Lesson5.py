"""
Chapter11_Lesson5.py
Time-varying controllability for a 2-state LTV system.

This script integrates the state transition matrix Phi(t,t0) and the
finite-horizon controllability Gramian Wc(t0,t) using the differential
Lyapunov equation

    dW/dt = A(t) W + W A(t)^T + B(t) B(t)^T,  W(t0)=0.

A positive definite Wc on [t0,tf] indicates complete controllability on
that interval for this continuous-time LTV system.
"""

import numpy as np
from scipy.integrate import solve_ivp


def A(t: float) -> np.ndarray:
    """Time-varying dynamics matrix."""
    return np.array([
        [0.0, 1.0],
        [-(2.0 + 0.60 * np.sin(t)), -0.25],
    ])


def B(t: float) -> np.ndarray:
    """Time-varying input matrix."""
    return np.array([
        [0.0],
        [1.0 + 0.45 * np.cos(t)],
    ])


def augmented_ode(t: float, z: np.ndarray, n: int) -> np.ndarray:
    """ODE for Phi(t,t0) and Wc(t0,t)."""
    Phi = z[: n * n].reshape(n, n)
    W = z[n * n :].reshape(n, n)
    At = A(t)
    Bt = B(t)

    dPhi = At @ Phi
    dW = At @ W + W @ At.T + Bt @ Bt.T
    return np.concatenate([dPhi.reshape(-1), dW.reshape(-1)])


def compute_gramian(t0: float, tf: float, n: int = 2):
    Phi0 = np.eye(n)
    W0 = np.zeros((n, n))
    z0 = np.concatenate([Phi0.reshape(-1), W0.reshape(-1)])

    sol = solve_ivp(
        fun=lambda t, z: augmented_ode(t, z, n),
        t_span=(t0, tf),
        y0=z0,
        rtol=1e-9,
        atol=1e-11,
        dense_output=False,
    )
    if not sol.success:
        raise RuntimeError(sol.message)

    zf = sol.y[:, -1]
    Phi_tf_t0 = zf[: n * n].reshape(n, n)
    Wc = zf[n * n :].reshape(n, n)
    Wc = 0.5 * (Wc + Wc.T)  # remove round-off asymmetry
    return Phi_tf_t0, Wc


def is_controllable(Wc: np.ndarray, tol: float = 1e-8) -> bool:
    eigvals = np.linalg.eigvalsh(Wc)
    return bool(np.min(eigvals) > tol)


if __name__ == "__main__":
    for tf in [0.25, 0.50, 1.00, 2.00, 4.00]:
        Phi, Wc = compute_gramian(0.0, tf)
        eigvals = np.linalg.eigvalsh(Wc)
        print(f"Interval [0, {tf:.2f}]")
        print("Phi(tf,t0)=\n", Phi)
        print("Wc(t0,tf)=\n", Wc)
        print("Eigenvalues of Wc:", eigvals)
        print("Controllable on this interval?", is_controllable(Wc))
        print("-" * 60)
