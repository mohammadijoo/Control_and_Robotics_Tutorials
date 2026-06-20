"""
Chapter30_Lesson3.py
Software tools for state-space analysis and design in Python.

Dependencies:
    pip install numpy scipy matplotlib control
Optional:
    pip install slycot
"""

import numpy as np
from numpy.linalg import matrix_rank, eigvals, cond
from scipy.linalg import expm, solve_continuous_are
from scipy.signal import StateSpace, lsim, place_poles
import matplotlib.pyplot as plt


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    for k in range(1, n):
        blocks.append(np.linalg.matrix_power(A, k) @ B)
    return np.hstack(blocks)


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return O = [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = [C]
    for k in range(1, n):
        blocks.append(C @ np.linalg.matrix_power(A, k))
    return np.vstack(blocks)


def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Continuous-time LQR gain K = R^(-1) B' P."""
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def finite_horizon_reachability_gramian(A: np.ndarray, B: np.ndarray, tf: float, steps: int = 4000) -> np.ndarray:
    """Numerically compute Wc(tf) = integral_0^tf exp(A t) B B' exp(A' t) dt."""
    n = A.shape[0]
    W = np.zeros((n, n))
    h = tf / steps
    for i in range(steps + 1):
        t = i * h
        Phi = expm(A * t)
        integrand = Phi @ B @ B.T @ Phi.T
        weight = 0.5 if i in (0, steps) else 1.0
        W += weight * integrand
    return h * W


def main() -> None:
    # Mass-spring-damper example: x1 = position, x2 = velocity
    m, c, k = 1.0, 0.35, 2.0
    A = np.array([[0.0, 1.0], [-k / m, -c / m]])
    B = np.array([[0.0], [1.0 / m]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    Ctrb = controllability_matrix(A, B)
    Obsv = observability_matrix(A, C)
    print("A eigenvalues:", eigvals(A))
    print("rank(Ctrb):", matrix_rank(Ctrb), "condition(Ctrb):", cond(Ctrb))
    print("rank(Obsv):", matrix_rank(Obsv), "condition(Obsv):", cond(Obsv))

    desired_poles = np.array([-2.0 + 1.5j, -2.0 - 1.5j])
    K_place = place_poles(A, B, desired_poles).gain_matrix
    print("Pole-placement K:", K_place)
    print("closed-loop poles:", eigvals(A - B @ K_place))

    Q = np.diag([20.0, 2.0])
    R = np.array([[0.5]])
    K_lqr = lqr_gain(A, B, Q, R)
    print("LQR K:", K_lqr)
    print("LQR poles:", eigvals(A - B @ K_lqr))

    Wc = finite_horizon_reachability_gramian(A, B, tf=5.0)
    print("Wc(5 s):\n", Wc)
    print("energy to reach xf=[1,0] approximately:", np.array([[1.0, 0.0]]) @ np.linalg.inv(Wc) @ np.array([[1.0], [0.0]]))

    # Closed-loop free response from an initial displacement.
    Acl = A - B @ K_lqr
    sys_cl = StateSpace(Acl, B, C, D)
    t = np.linspace(0.0, 8.0, 600)
    u = np.zeros_like(t)
    tout, y, x = lsim(sys_cl, U=u, T=t, X0=np.array([1.0, 0.0]))

    plt.figure()
    plt.plot(tout, x[:, 0], label="position")
    plt.plot(tout, x[:, 1], label="velocity")
    plt.xlabel("time [s]")
    plt.ylabel("state")
    plt.title("Chapter 30 Lesson 3: LQR closed-loop response")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
