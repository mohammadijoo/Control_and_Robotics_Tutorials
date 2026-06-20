"""
Chapter21_Lesson4.py
Minimum-phase vs non-minimum-phase systems in state-space.

This script computes transmission zeros from a SISO state-space model,
classifies the system, and constructs the relative-degree-one zero dynamics.
Dependencies: numpy, scipy.
"""

import numpy as np
from numpy.linalg import eigvals
from scipy.signal import ss2tf, StateSpace, step
from scipy.linalg import null_space


def siso_zeros_from_state_space(A, B, C, D, tol=1e-9):
    """Return finite zeros of a SISO state-space model using ss2tf."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    D = np.asarray(D, dtype=float)

    num, den = ss2tf(A, B, C, D)
    num = np.trim_zeros(num[0], trim="f")
    if len(num) == 0:
        return np.array([]), num, den

    # Remove nearly zero leading coefficients caused by numerical roundoff.
    while len(num) > 1 and abs(num[0]) < tol:
        num = num[1:]
    zeros = np.roots(num) if len(num) > 1 else np.array([])
    return zeros, num, den


def classify_continuous_time(zeros, tol=1e-8):
    """Classify continuous-time finite zeros by real part."""
    zeros = np.asarray(zeros, dtype=complex)
    if zeros.size == 0:
        return "minimum-phase (no finite transmission zeros)"
    if np.any(np.real(zeros) > tol):
        return "non-minimum-phase (at least one right-half-plane zero)"
    if np.any(np.abs(np.real(zeros)) <= tol):
        return "borderline non-minimum-phase (zero on imaginary axis)"
    return "minimum-phase (all finite zeros in the open left-half-plane)"


def zero_dynamics_rd1(A, B, C, tol=1e-10):
    """
    Relative-degree-one SISO zero dynamics for D=0 and C B != 0.

    y = C x = 0, so x = N z with columns of N spanning ker(C).
    Enforce dot(y) = C A x + C B u = 0, hence
    u = -(C B)^(-1) C A N z.
    Then dot(z) = L (I - B(CB)^(-1)C) A N z, where L N = I.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)

    CB = C @ B
    if abs(CB.item()) < tol:
        raise ValueError("This formula requires relative degree one: C B must be nonzero.")

    N = null_space(C)              # n x (n-1), orthonormal basis for ker(C)
    L = N.T                        # since scipy returns orthonormal columns
    projection = np.eye(A.shape[0]) - B @ np.linalg.inv(CB) @ C
    A0 = L @ projection @ A @ N
    return A0, eigvals(A0), N


def print_analysis(name, A, B, C, D):
    print("\n" + "=" * 72)
    print(name)
    print("A eigenvalues (poles):", eigvals(A))
    zeros, num, den = siso_zeros_from_state_space(A, B, C, D)
    print("Transfer numerator coefficients:", num)
    print("Transfer denominator coefficients:", den)
    print("Transmission zeros:", zeros)
    print("Classification:", classify_continuous_time(zeros))

    if abs(float(D)) < 1e-12:
        try:
            A0, zd_eigs, _ = zero_dynamics_rd1(A, B, C)
            print("Zero-dynamics matrix A0:\n", A0)
            print("Zero-dynamics eigenvalues:", zd_eigs)
        except ValueError as exc:
            print("Zero-dynamics rd=1 construction skipped:", exc)


def main():
    # Two stable second-order plants with the same poles and opposite zero location.
    # G_min(s) = (s + 1) / ((s + 2)(s + 3))
    A_min = np.array([[0.0, 1.0], [-6.0, -5.0]])
    B_min = np.array([[0.0], [1.0]])
    C_min = np.array([[1.0, 1.0]])
    D_min = np.array([[0.0]])

    # G_nmp(s) = (-s + 1) / ((s + 2)(s + 3)); zero at +1, DC gain positive.
    A_nmp = np.array([[0.0, 1.0], [-6.0, -5.0]])
    B_nmp = np.array([[0.0], [1.0]])
    C_nmp = np.array([[1.0, -1.0]])
    D_nmp = np.array([[0.0]])

    print_analysis("Minimum-phase example", A_min, B_min, C_min, D_min)
    print_analysis("Non-minimum-phase example", A_nmp, B_nmp, C_nmp, D_nmp)

    # Optional step response data for plotting outside this script.
    t = np.linspace(0.0, 8.0, 300)
    _, y_min = step(StateSpace(A_min, B_min, C_min, D_min), T=t)
    _, y_nmp = step(StateSpace(A_nmp, B_nmp, C_nmp, D_nmp), T=t)
    print("\nFirst five step samples for minimum-phase plant:", y_min[:5])
    print("First five step samples for non-minimum-phase plant:", y_nmp[:5])
    print("The NMP response initially moves in the opposite direction because of the RHP zero.")


if __name__ == "__main__":
    main()
