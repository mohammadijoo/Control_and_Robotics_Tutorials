# Chapter10_Lesson3.py
# Finite-time reachability and minimum-energy steering for continuous-time LTI systems.
# Requires: numpy, scipy

import numpy as np
from scipy.linalg import expm
from scipy.integrate import quad_vec


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Apow = np.eye(n)
    for _ in range(n):
        blocks.append(Apow @ B)
        Apow = Apow @ A
    return np.hstack(blocks)


def finite_time_gramian(A: np.ndarray, B: np.ndarray, T: float) -> np.ndarray:
    """Compute W(T)=int_0^T exp(A s) B B' exp(A' s) ds."""
    if T <= 0:
        raise ValueError("T must be positive.")

    def integrand(s):
        E = expm(A * s)
        return E @ B @ B.T @ E.T

    W, _ = quad_vec(integrand, 0.0, T)
    return 0.5 * (W + W.T)


def min_energy_control(A: np.ndarray, B: np.ndarray, x0: np.ndarray, xT: np.ndarray, T: float):
    """Return a callable u(t), the Gramian W(T), displacement d, and minimum energy."""
    PhiT = expm(A * T)
    W = finite_time_gramian(A, B, T)
    d = xT - PhiT @ x0
    Winv = np.linalg.pinv(W)

    if np.linalg.norm(W @ Winv @ d - d) > 1e-8:
        raise ValueError("The requested terminal state is not reachable over this horizon.")

    def u(t):
        return B.T @ expm(A.T * (T - t)) @ Winv @ d

    energy = float(d.T @ Winv @ d)
    return u, W, d, energy


def simulate_closed_form(A, B, x0, u, T, steps=400):
    """Approximate x(T)=Phi(T)x0+int_0^T Phi(T-t)B u(t) dt by trapezoidal integration."""
    ts = np.linspace(0.0, T, steps + 1)
    total = np.zeros_like(x0, dtype=float)
    for k, t in enumerate(ts):
        weight = 0.5 if k == 0 or k == len(ts) - 1 else 1.0
        total += weight * (expm(A * (T - t)) @ B @ np.atleast_1d(u(t)))
    total *= T / steps
    return expm(A * T) @ x0 + total


def demo():
    # Double integrator: x1_dot = x2, x2_dot = u.
    A = np.array([[0.0, 1.0],
                  [0.0, 0.0]])
    B = np.array([[0.0],
                  [1.0]])
    x0 = np.array([0.0, 0.0])
    xT = np.array([1.0, 0.0])

    C = controllability_matrix(A, B)
    print("Controllability matrix:\n", C)
    print("rank =", np.linalg.matrix_rank(C))

    for T in [0.5, 1.0, 2.0, 4.0]:
        u, W, d, energy = min_energy_control(A, B, x0, xT, T)
        xT_num = simulate_closed_form(A, B, x0, u, T)
        print(f"\nT = {T:.2f}")
        print("W(T) =\n", W)
        print("det(W) =", np.linalg.det(W))
        print("minimum energy =", energy)
        print("reconstructed x(T) =", xT_num)
        print("sample u(0), u(T/2), u(T) =", u(0.0), u(T / 2.0), u(T))


if __name__ == "__main__":
    demo()
