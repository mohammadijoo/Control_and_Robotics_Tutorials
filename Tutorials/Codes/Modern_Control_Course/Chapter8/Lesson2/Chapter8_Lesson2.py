"""
Chapter8_Lesson2.py
Modern Control — Chapter 8, Lesson 2
Semigroup Property and Inverse of the State Transition Matrix Phi(t)

Requires: numpy, scipy
Install if needed: pip install numpy scipy
"""

import numpy as np
from scipy.linalg import expm, norm


def phi(A: np.ndarray, t: float) -> np.ndarray:
    """State transition matrix Phi(t) = exp(A t)."""
    return expm(A * t)


def main() -> None:
    # A stable second-order continuous-time state matrix.
    A = np.array([[0.0, 1.0],
                  [-4.0, -0.8]])

    t = 0.7
    s = 1.3
    x0 = np.array([1.0, -0.25])

    Phi_t = phi(A, t)
    Phi_s = phi(A, s)
    Phi_t_plus_s = phi(A, t + s)
    Phi_minus_t = phi(A, -t)

    # Semigroup property: Phi(t+s) = Phi(t) Phi(s).
    semigroup_error = norm(Phi_t_plus_s - Phi_t @ Phi_s, ord="fro")

    # Inverse property: Phi(t)^(-1) = Phi(-t).
    inverse_error = norm(np.linalg.inv(Phi_t) - Phi_minus_t, ord="fro")
    identity_error = norm(Phi_t @ Phi_minus_t - np.eye(A.shape[0]), ord="fro")

    # State propagation equivalence.
    x_direct = Phi_t_plus_s @ x0
    x_two_step = Phi_t @ (Phi_s @ x0)
    state_error = norm(x_direct - x_two_step)

    print("A =")
    print(A)
    print("\nPhi(t) =")
    print(Phi_t)
    print("\nPhi(s) =")
    print(Phi_s)
    print("\nPhi(t+s) =")
    print(Phi_t_plus_s)
    print("\nSemigroup error ||Phi(t+s)-Phi(t)Phi(s)||_F =", semigroup_error)
    print("Inverse error ||inv(Phi(t))-Phi(-t)||_F =", inverse_error)
    print("Identity error ||Phi(t)Phi(-t)-I||_F =", identity_error)
    print("State two-step propagation error =", state_error)


if __name__ == "__main__":
    main()
