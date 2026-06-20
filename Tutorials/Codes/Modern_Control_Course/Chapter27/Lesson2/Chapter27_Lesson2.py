# Chapter27_Lesson2.py
# Feedforward gain design for reference tracking in continuous-time state space.
# Requirements: numpy, scipy, matplotlib (optional plotting)

import numpy as np
from numpy.linalg import inv, matrix_rank, solve, eigvals
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def feedforward_from_dc_gain(A, B, C, D, K, tol=1e-9):
    """Return N such that the closed-loop DC gain from r to y is identity.

    Plant:  xdot = A x + B u, y = C x + D u
    Control: u = -K x + N r

    G0 = D - (C - D K) (A - B K)^(-1) B
    Choose N = G0^(-1), when G0 is square and nonsingular.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    D = np.asarray(D, dtype=float)
    K = np.asarray(K, dtype=float)

    Acl = A - B @ K
    Ccl = C - D @ K
    G0 = D - Ccl @ solve(Acl, B)

    if G0.shape[0] != G0.shape[1]:
        raise ValueError("G0 must be square for direct inversion. Use regulator equations instead.")
    if matrix_rank(G0, tol=tol) < G0.shape[0]:
        raise ValueError("Closed-loop DC gain is singular; exact static feedforward tracking is impossible.")
    return inv(G0), G0


def feedforward_from_regulator_equations(A, B, C, D, K, tol=1e-9):
    """Solve the steady-state regulator equations.

    [A B] [X] = [0]
    [C D] [U]   [I]

    Then N = U + K X, because u_ss = -K x_ss + N r.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    D = np.asarray(D, dtype=float)
    K = np.asarray(K, dtype=float)

    n = A.shape[0]
    m = B.shape[1]
    p = C.shape[0]
    R = np.block([[A, B], [C, D]])
    RHS = np.vstack([np.zeros((n, p)), np.eye(p)])

    if matrix_rank(R, tol=tol) < n + p:
        raise ValueError("Rosenbrock matrix at s=0 has deficient rank; origin zero or insufficient authority.")

    # Least-squares also handles over-actuated cases. For square nonsingular R, it equals solve(R,RHS).
    sol, *_ = np.linalg.lstsq(R, RHS, rcond=None)
    X = sol[:n, :]
    U = sol[n:n + m, :]
    N = U + K @ X
    return N, X, U


def simulate_tracking(A, B, C, D, K, N, r, t_final=8.0, x0=None):
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    D = np.asarray(D, dtype=float)
    K = np.asarray(K, dtype=float)
    N = np.asarray(N, dtype=float)
    r = np.asarray(r, dtype=float).reshape(-1, 1)
    n = A.shape[0]
    if x0 is None:
        x0 = np.zeros(n)

    def rhs(t, x):
        x_col = x.reshape(-1, 1)
        u = -K @ x_col + N @ r
        return (A @ x_col + B @ u).ravel()

    t_eval = np.linspace(0.0, t_final, 800)
    sol = solve_ivp(rhs, [0.0, t_final], x0, t_eval=t_eval, rtol=1e-8, atol=1e-10)
    X = sol.y
    U = np.zeros((B.shape[1], len(sol.t)))
    Y = np.zeros((C.shape[0], len(sol.t)))
    for i in range(len(sol.t)):
        x_col = X[:, [i]]
        u = -K @ x_col + N @ r
        y = C @ x_col + D @ u
        U[:, i] = u.ravel()
        Y[:, i] = y.ravel()
    return sol.t, X, U, Y


if __name__ == "__main__":
    # Example: second-order position servo with state x=[position, velocity]^T.
    A = np.array([[0.0, 1.0],
                  [0.0, 0.0]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    # K gives closed-loop characteristic polynomial s^2 + 4s + 4 = (s+2)^2.
    K = np.array([[4.0, 4.0]])

    N_dc, G0 = feedforward_from_dc_gain(A, B, C, D, K)
    N_reg, Xss, Uss = feedforward_from_regulator_equations(A, B, C, D, K)

    print("Closed-loop eigenvalues:", eigvals(A - B @ K))
    print("G0 =", G0)
    print("N from DC gain =", N_dc)
    print("N from regulator equations =", N_reg)
    print("X steady-state map =", Xss)
    print("U steady-state map =", Uss)

    t, X, U, Y = simulate_tracking(A, B, C, D, K, N_dc, r=[1.0], t_final=6.0)
    print("Final output y(t_final) =", Y[:, -1])

    plt.figure()
    plt.plot(t, Y[0, :], label="y: position")
    plt.axhline(1.0, linestyle="--", label="reference")
    plt.xlabel("time [s]")
    plt.ylabel("output")
    plt.title("Reference tracking with static feedforward gain")
    plt.grid(True)
    plt.legend()
    plt.show()
