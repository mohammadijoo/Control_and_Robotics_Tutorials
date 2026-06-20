"""
Chapter28_Lesson3.py
Modern Control — Chapter 28, Lesson 3
State and control weighting matrices Q and R as performance descriptors.

This script demonstrates:
1. Symmetry and definiteness checks for Q and R.
2. Bryson-style diagonal weight selection.
3. Finite-horizon weighted performance integral for a simulated state/input record.
4. Coordinate-transformation consistency of a quadratic state cost.
"""

from __future__ import annotations

import numpy as np
from numpy.linalg import eigvalsh
from scipy.integrate import solve_ivp


def symmetrize(M: np.ndarray) -> np.ndarray:
    """Return the symmetric part of a square matrix."""
    return 0.5 * (M + M.T)


def is_psd(M: np.ndarray, tol: float = 1e-10) -> bool:
    """Check positive semidefiniteness through symmetric eigenvalues."""
    Ms = symmetrize(M)
    return bool(np.min(eigvalsh(Ms)) >= -tol)


def is_pd(M: np.ndarray, tol: float = 1e-10) -> bool:
    """Check positive definiteness through symmetric eigenvalues."""
    Ms = symmetrize(M)
    return bool(np.min(eigvalsh(Ms)) > tol)


def bryson_weights(max_abs_values: np.ndarray) -> np.ndarray:
    """
    Bryson-style diagonal weighting:
        weight_i = 1 / allowed_i^2.
    The vector contains acceptable maximum magnitudes of variables.
    """
    max_abs_values = np.asarray(max_abs_values, dtype=float)
    if np.any(max_abs_values <= 0.0):
        raise ValueError("All allowed magnitudes must be positive.")
    return np.diag(1.0 / (max_abs_values**2))


def simulate_closed_loop(A: np.ndarray, B: np.ndarray, K: np.ndarray, x0: np.ndarray, tf: float = 8.0):
    """Simulate xdot = (A - B K) x and u = -K x."""
    Acl = A - B @ K

    def rhs(_t, x):
        return Acl @ x

    sol = solve_ivp(rhs, (0.0, tf), x0, dense_output=False, max_step=0.01, rtol=1e-8, atol=1e-10)
    X = sol.y.T
    U = -(K @ X.T).T
    return sol.t, X, U


def weighted_cost(t: np.ndarray, X: np.ndarray, U: np.ndarray, Q: np.ndarray, R: np.ndarray) -> float:
    """Approximate integral of x'Qx + u'Ru using the trapezoidal rule."""
    state_terms = np.einsum("bi,ij,bj->b", X, Q, X)
    input_terms = np.einsum("bi,ij,bj->b", U, R, U)
    return float(np.trapz(state_terms + input_terms, t))


def main() -> None:
    # A stable feedback example for a second-order plant.
    A = np.array([[0.0, 1.0], [-2.0, -0.4]])
    B = np.array([[0.0], [1.0]])
    K = np.array([[3.0, 2.2]])
    x0 = np.array([1.0, 0.0])

    # Performance-descriptor weights from engineering tolerances.
    # State 1 allowed magnitude: 1.0; state 2 allowed magnitude: 2.0.
    Q = bryson_weights(np.array([1.0, 2.0]))
    # Input allowed magnitude: 0.5.
    R = bryson_weights(np.array([0.5]))

    print("Q =\n", Q)
    print("R =\n", R)
    print("Q is PSD:", is_psd(Q))
    print("R is PD:", is_pd(R))

    t, X, U = simulate_closed_loop(A, B, K, x0)
    J = weighted_cost(t, X, U, Q, R)
    print(f"Finite-horizon weighted cost J_T = {J:.6f}")

    # Coordinate transformation: x = T z.
    T = np.array([[2.0, 0.5], [0.0, 1.5]])
    Z = np.linalg.solve(T, X.T).T
    Qz = T.T @ Q @ T
    J_state_x = float(np.trapz(np.einsum("bi,ij,bj->b", X, Q, X), t))
    J_state_z = float(np.trapz(np.einsum("bi,ij,bj->b", Z, Qz, Z), t))
    print(f"State cost in x-coordinates = {J_state_x:.6f}")
    print(f"State cost in z-coordinates = {J_state_z:.6f}")
    print("Coordinate consistency error:", abs(J_state_x - J_state_z))


if __name__ == "__main__":
    main()
