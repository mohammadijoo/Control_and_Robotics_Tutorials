# Chapter28_Lesson4.py
# Qualitative effect of Q and R weights on closed-loop behavior.
#
# Libraries:
#   numpy, scipy, matplotlib
# Optional modern-control ecosystem:
#   python-control can also compute lqr(A, B, Q, R), but this file uses SciPy directly.

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp


def lqr_gain(A, B, Q, R):
    """Continuous-time LQR gain K = R^{-1} B^T P."""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    eig_cl = np.linalg.eigvals(A - B @ K)
    return K, P, eig_cl


def simulate_closed_loop(A, B, K, x0, t_final=8.0):
    """Simulate x_dot = (A - B K) x from an initial condition."""
    Acl = A - B @ K

    def rhs(t, x):
        return Acl @ x

    sol = solve_ivp(rhs, (0.0, t_final), x0, max_step=0.01, dense_output=True)
    t = np.linspace(0.0, t_final, 900)
    X = sol.sol(t)
    U = -K @ X
    return t, X, U


def main():
    # A lightly damped second-order system:
    # x1 = position-like state, x2 = velocity-like state, u = force-like input.
    A = np.array([[0.0, 1.0],
                  [-1.0, -0.15]])
    B = np.array([[0.0],
                  [1.0]])

    cases = {
        "balanced": (np.diag([1.0, 1.0]), np.array([[1.0]])),
        "high_position_weight": (np.diag([25.0, 1.0]), np.array([[1.0]])),
        "high_velocity_weight": (np.diag([1.0, 25.0]), np.array([[1.0]])),
        "high_input_penalty": (np.diag([1.0, 1.0]), np.array([[25.0]])),
        "low_input_penalty": (np.diag([1.0, 1.0]), np.array([[0.04]])),
    }

    x0 = np.array([1.0, 0.0])
    summary = []

    plt.figure(figsize=(10, 6))
    for name, (Q, R) in cases.items():
        K, P, eig_cl = lqr_gain(A, B, Q, R)
        t, X, U = simulate_closed_loop(A, B, K, x0)
        J0 = float(x0.T @ P @ x0)
        summary.append((name, K, eig_cl, J0, np.max(np.abs(U))))
        plt.plot(t, X[0, :], label=f"{name}: x1")

    plt.xlabel("time [s]")
    plt.ylabel("state x1")
    plt.title("Changing Q and R reshapes the closed-loop transient after redesign")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    print("\nQualitative tuning table")
    print("-" * 92)
    print(f"{'case':25s} {'K':27s} {'closed-loop eigenvalues':30s} {'J(x0)':>8s} {'max|u|':>8s}")
    print("-" * 92)
    for name, K, eig_cl, J0, umax in summary:
        k_str = np.array2string(K, precision=3, suppress_small=True)
        e_str = np.array2string(eig_cl, precision=3, suppress_small=True)
        print(f"{name:25s} {k_str:27s} {e_str:30s} {J0:8.3f} {umax:8.3f}")


if __name__ == "__main__":
    main()
