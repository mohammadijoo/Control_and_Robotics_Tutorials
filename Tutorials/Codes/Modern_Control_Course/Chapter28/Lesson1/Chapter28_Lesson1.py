"""
Chapter28_Lesson1.py

Quadratic forms in state and input variables for modern control.

This script demonstrates:
1. Symmetry and definiteness checks for Q and R.
2. Evaluation of state/input quadratic performance density.
3. Completing the square for a cross term x^T Q x + 2 x^T N u + u^T R u.
4. Numerical integration of a quadratic performance measure along a closed-loop trajectory.

Recommended libraries:
    pip install numpy scipy matplotlib
Optional control library:
    pip install control
"""

import numpy as np
from scipy.integrate import solve_ivp


def symmetrize(M: np.ndarray) -> np.ndarray:
    """Return the symmetric part of M because z^T M z only depends on (M+M^T)/2."""
    return 0.5 * (M + M.T)


def eig_definiteness(M: np.ndarray, tol: float = 1e-10) -> str:
    """Classify a real symmetric matrix using its eigenvalues."""
    S = symmetrize(M)
    eigs = np.linalg.eigvalsh(S)
    if np.all(eigs > tol):
        return "positive definite"
    if np.all(eigs >= -tol):
        return "positive semidefinite"
    if np.all(eigs < -tol):
        return "negative definite"
    if np.all(eigs <= tol):
        return "negative semidefinite"
    return "indefinite"


def quadratic_form(z: np.ndarray, M: np.ndarray) -> float:
    """Compute z^T M z."""
    z = np.asarray(z, dtype=float).reshape(-1, 1)
    return float(z.T @ M @ z)


def stage_cost(x: np.ndarray, u: np.ndarray, Q: np.ndarray, R: np.ndarray, N: np.ndarray | None = None) -> float:
    """Compute l(x,u)=x^T Q x + 2 x^T N u + u^T R u."""
    x = np.asarray(x, dtype=float).reshape(-1, 1)
    u = np.asarray(u, dtype=float).reshape(-1, 1)
    value = float(x.T @ Q @ x + u.T @ R @ u)
    if N is not None:
        value += float(2.0 * x.T @ N @ u)
    return value


def complete_square(Q: np.ndarray, R: np.ndarray, N: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    For R positive definite:
        x^T Q x + 2 x^T N u + u^T R u
      = (u + R^{-1} N^T x)^T R (u + R^{-1} N^T x)
        + x^T (Q - N R^{-1} N^T) x.
    """
    Rinv_NT = np.linalg.solve(R, N.T)
    Schur = Q - N @ Rinv_NT
    return Rinv_NT, Schur


def simulate_closed_loop_cost(A: np.ndarray, B: np.ndarray, K: np.ndarray,
                              Q: np.ndarray, R: np.ndarray,
                              x0: np.ndarray, tf: float = 10.0) -> float:
    """
    Simulate x_dot = (A-BK)x, u=-Kx, and integrate J = int_0^tf (x^T Q x + u^T R u) dt.
    The augmented state is [x; J].
    """
    n = A.shape[0]

    def rhs(t, z):
        x = z[:n].reshape(n, 1)
        u = -K @ x
        dx = (A @ x + B @ u).ravel()
        dJ = stage_cost(x, u, Q, R)
        return np.r_[dx, dJ]

    z0 = np.r_[x0.ravel(), 0.0]
    sol = solve_ivp(rhs, [0.0, tf], z0, rtol=1e-9, atol=1e-11)
    return float(sol.y[-1, -1])


if __name__ == "__main__":
    # Two-state oscillator-like plant: x=[position, velocity]^T
    A = np.array([[0.0, 1.0],
                  [-2.0, -0.4]])
    B = np.array([[0.0],
                  [1.0]])

    Q = np.diag([10.0, 1.0])     # penalize position more than velocity
    R = np.array([[0.25]])       # penalize input effort
    N = np.array([[0.0],
                  [0.15]])      # optional state-input cross weight

    x = np.array([0.7, -0.2])
    u = np.array([0.4])

    print("Q is", eig_definiteness(Q))
    print("R is", eig_definiteness(R))
    print("x^T Q x =", quadratic_form(x, Q))
    print("stage cost with cross term =", stage_cost(x, u, Q, R, N))

    shift, schur = complete_square(Q, R, N)
    print("R^{-1} N^T =")
    print(shift)
    print("Schur complement Q - N R^{-1} N^T =")
    print(schur)
    print("Schur complement is", eig_definiteness(schur))

    # A stabilizing feedback gain chosen for demonstration, not yet derived optimally.
    K = np.array([[3.0, 2.0]])
    x0 = np.array([1.0, 0.0])
    J = simulate_closed_loop_cost(A, B, K, Q, R, x0, tf=8.0)
    print("Finite-horizon quadratic performance J =", J)
