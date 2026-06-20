# Chapter8_Lesson1.py
# Modern Control — Chapter 8, Lesson 1: Definition of the State Transition Matrix
#
# Requires: numpy, scipy, matplotlib (optional)
#   pip install numpy scipy matplotlib
#
# This script:
#   1) Computes Phi(t,t0) = expm(A*(t-t0))
#   2) Propagates x(t) = Phi(t,t0) x0
#   3) Verifies against numerical integration of xdot = A x

import numpy as np
from scipy.linalg import expm, norm
from scipy.integrate import solve_ivp

def state_transition_matrix(A: np.ndarray, t: float, t0: float = 0.0) -> np.ndarray:
    """Phi(t,t0) for LTI xdot = A x."""
    return expm(A * (t - t0))

def propagate_state(A: np.ndarray, x0: np.ndarray, t: float, t0: float = 0.0) -> np.ndarray:
    Phi = state_transition_matrix(A, t, t0)
    return Phi @ x0

def integrate_state(A: np.ndarray, x0: np.ndarray, t: float, t0: float = 0.0) -> np.ndarray:
    """Numerical integration for comparison."""
    def f(_tau, x):
        return (A @ x)

    sol = solve_ivp(f, (t0, t), x0, method="RK45", rtol=1e-10, atol=1e-12)
    return sol.y[:, -1]

def main():
    # Example A (stable 2x2) and initial condition
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    x0 = np.array([1.0, -0.5])

    t0 = 0.0
    t  = 1.25

    Phi = state_transition_matrix(A, t, t0)
    x_analytic = propagate_state(A, x0, t, t0)
    x_numeric  = integrate_state(A, x0, t, t0)

    print("A =\n", A)
    print("t0 =", t0, "t =", t)
    print("Phi(t,t0) =\n", Phi)
    print("x_analytic =", x_analytic)
    print("x_numeric  =", x_numeric)
    print("||x_analytic - x_numeric||_2 =", norm(x_analytic - x_numeric, 2))

    # Optional: check the defining ODE for Phi(t,t0): d/dt Phi = A Phi
    dt = 1e-6
    Phi_t   = state_transition_matrix(A, t, t0)
    Phi_tdt = state_transition_matrix(A, t + dt, t0)
    Phi_dot_fd = (Phi_tdt - Phi_t) / dt
    residual = Phi_dot_fd - A @ Phi_t
    print("||Phi_dot_fd - A Phi||_F =", norm(residual, ord="fro"))

if __name__ == "__main__":
    main()
