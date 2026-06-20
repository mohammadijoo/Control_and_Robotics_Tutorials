"""
Chapter22_Lesson2.py
Closed-Loop State Matrix and Mode Relocation

Topic:
    For continuous-time LTI dynamics x_dot = A x + B u and full-state feedback
    u = -K x + r, the autonomous closed-loop state matrix is Acl = A - B K.

This script demonstrates:
    1. Open-loop and closed-loop eigenvalue computation.
    2. Mode relocation by selecting K.
    3. Optional pole placement using scipy.signal.place_poles.
    4. Verification of the characteristic polynomial.
    5. Numerical simulation of x_dot = (A - B K) x.

Libraries:
    numpy              : matrix algebra
    scipy.linalg       : matrix exponential
    scipy.signal       : pole placement, if SciPy is installed
    matplotlib         : optional plotting
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

try:
    from scipy.linalg import expm
    from scipy.signal import place_poles
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False


def closed_loop_matrix(A: NDArray[np.float64],
                       B: NDArray[np.float64],
                       K: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return Acl = A - B K."""
    return A - B @ K


def characteristic_coefficients_2x2(A: NDArray[np.float64]) -> tuple[float, float]:
    """
    For a 2 x 2 matrix A, return coefficients a1, a0 of
        s^2 + a1 s + a0.
    Since det(sI - A) = s^2 - tr(A) s + det(A),
        a1 = -tr(A), a0 = det(A).
    """
    trace = float(np.trace(A))
    determinant = float(np.linalg.det(A))
    return -trace, determinant


def second_order_feedback_by_matching(A: NDArray[np.float64],
                                      B: NDArray[np.float64],
                                      desired_poles: list[complex]) -> NDArray[np.float64]:
    """
    From-scratch gain computation for a controllable second-order SISO system.

    This function solves for K = [k1 k2] by matching
        det(sI - (A - B K)) = (s - p1)(s - p2).

    It numerically solves two nonlinear polynomial-coefficient equations by
    exploiting that the coefficients are affine in k1, k2 for SISO 2x2 systems.
    """
    if A.shape != (2, 2) or B.shape != (2, 1):
        raise ValueError("This scratch implementation is only for 2x2 SISO systems.")

    p1, p2 = desired_poles
    desired_a1 = float(np.real(-(p1 + p2)))
    desired_a0 = float(np.real(p1 * p2))

    def coeffs_for(k1: float, k2: float) -> NDArray[np.float64]:
        K = np.array([[k1, k2]], dtype=float)
        Acl = closed_loop_matrix(A, B, K)
        a1, a0 = characteristic_coefficients_2x2(Acl)
        return np.array([a1, a0], dtype=float)

    c00 = coeffs_for(0.0, 0.0)
    c10 = coeffs_for(1.0, 0.0)
    c01 = coeffs_for(0.0, 1.0)

    # coeffs(k1,k2) = c00 + M [k1 k2]^T
    M = np.column_stack([c10 - c00, c01 - c00])
    target = np.array([desired_a1, desired_a0], dtype=float)
    k = np.linalg.solve(M, target - c00)
    return k.reshape(1, 2)


def simulate_homogeneous(Acl: NDArray[np.float64],
                         x0: NDArray[np.float64],
                         t_grid: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return x(t) = exp(Acl t) x0 for all t in t_grid."""
    if not SCIPY_AVAILABLE:
        raise RuntimeError("scipy.linalg.expm is required for simulation.")
    states = []
    for t in t_grid:
        states.append(expm(Acl * t) @ x0)
    return np.asarray(states)


def main() -> None:
    # Mass-spring-damper-like second-order model in phase-variable form:
    # x1_dot = x2
    # x2_dot = -2 x1 - 0.4 x2 + u
    A = np.array([[0.0, 1.0],
                  [-2.0, -0.4]])
    B = np.array([[0.0],
                  [1.0]])

    open_loop_modes = np.linalg.eigvals(A)
    print("Open-loop A:")
    print(A)
    print("Open-loop modes:", open_loop_modes)

    desired_poles = [-2.0, -3.0]

    # From-scratch coefficient matching for 2x2 SISO.
    K_scratch = second_order_feedback_by_matching(A, B, desired_poles)
    Acl_scratch = closed_loop_matrix(A, B, K_scratch)

    print("\nK from 2x2 coefficient matching:")
    print(K_scratch)
    print("Acl = A - B K:")
    print(Acl_scratch)
    print("Closed-loop modes:", np.linalg.eigvals(Acl_scratch))
    print("Closed-loop characteristic coefficients:",
          characteristic_coefficients_2x2(Acl_scratch))

    # Optional library method.
    if SCIPY_AVAILABLE:
        result = place_poles(A, B, desired_poles)
        K_library = result.gain_matrix
        Acl_library = closed_loop_matrix(A, B, K_library)
        print("\nK from scipy.signal.place_poles:")
        print(K_library)
        print("Closed-loop modes:", np.linalg.eigvals(Acl_library))

        t = np.linspace(0.0, 5.0, 201)
        x0 = np.array([1.0, 0.0])
        X = simulate_homogeneous(Acl_library, x0, t)
        print("\nSample trajectory values [t, x1, x2]:")
        for idx in [0, 25, 50, 100, 200]:
            print(f"{t[idx]:.2f}, {X[idx,0]: .6f}, {X[idx,1]: .6f}")

        # Optional plotting. Uncomment if running locally.
        # import matplotlib.pyplot as plt
        # plt.plot(t, X[:, 0], label="x1")
        # plt.plot(t, X[:, 1], label="x2")
        # plt.xlabel("time")
        # plt.ylabel("state")
        # plt.legend()
        # plt.grid(True)
        # plt.show()
    else:
        print("\nSciPy is not available; library pole placement and simulation skipped.")


if __name__ == "__main__":
    main()
