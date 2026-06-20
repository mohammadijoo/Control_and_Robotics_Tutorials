"""
Chapter28_Lesson5.py

Preview of optimal state-feedback for the continuous-time LQR problem.

System:
    x_dot = A x + B u

Cost:
    J = integral_0^infty (x.T Q x + u.T R u) dt

The script solves the continuous-time algebraic Riccati equation (CARE),
computes the feedback gain K = R^{-1} B.T P, simulates the closed-loop
system, and estimates the finite-horizon approximation of the cost.
"""

import numpy as np
from numpy.linalg import inv, eigvals
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp


def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray):
    """Return P and K for the continuous-time LQR problem."""
    P = solve_continuous_are(A, B, Q, R)
    K = inv(R) @ B.T @ P
    return P, K


def simulate_closed_loop(A, B, K, Q, R, x0, tf=8.0, npts=1200):
    """Simulate x_dot = (A - B K) x and estimate the quadratic cost."""
    Acl = A - B @ K

    def rhs(t, x):
        return Acl @ x

    t_eval = np.linspace(0.0, tf, npts)
    sol = solve_ivp(rhs, (0.0, tf), x0, t_eval=t_eval, rtol=1e-9, atol=1e-11)

    X = sol.y.T
    U = -(K @ X.T).T
    running_cost = np.einsum("bi,ij,bj->b", X, Q, X) + np.einsum(
        "bi,ij,bj->b", U, R, U
    )
    J_est = np.trapz(running_cost, sol.t)
    return sol.t, X, U, J_est


def main():
    # A standard mass-normalized double integrator.
    A = np.array([[0.0, 1.0],
                  [0.0, 0.0]])
    B = np.array([[0.0],
                  [1.0]])

    # Position is penalized more than velocity; control is relatively cheap.
    Q = np.diag([10.0, 1.0])
    R = np.array([[0.2]])

    P, K = lqr_gain(A, B, Q, R)
    Acl = A - B @ K

    x0 = np.array([1.0, 0.0])
    t, X, U, J_est = simulate_closed_loop(A, B, K, Q, R, x0)

    print("P =")
    print(P)
    print("\nK =")
    print(K)
    print("\nclosed-loop eigenvalues =")
    print(eigvals(Acl))
    print("\nfinite-horizon estimated cost over [0, 8] =", J_est)
    print("infinite-horizon value x0.T P x0 =", x0 @ P @ x0)

    try:
        import matplotlib.pyplot as plt

        plt.figure()
        plt.plot(t, X[:, 0], label="position")
        plt.plot(t, X[:, 1], label="velocity")
        plt.plot(t, U[:, 0], label="control")
        plt.xlabel("time")
        plt.grid(True)
        plt.legend()
        plt.title("Chapter 28 Lesson 5: LQR closed-loop response")
        plt.show()
    except Exception as exc:
        print("Plot skipped:", exc)


if __name__ == "__main__":
    main()
