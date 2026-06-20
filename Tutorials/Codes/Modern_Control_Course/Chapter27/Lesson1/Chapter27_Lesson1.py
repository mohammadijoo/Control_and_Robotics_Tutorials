# Chapter27_Lesson1.py
# Formulating Tracking Problems in State-Space Form
#
# This script formulates a continuous-time tracking problem
#
#   x_dot = A x + B u + E d
#   y     = C x + D u + F d
#   e     = y - r
#
# It computes the steady-state equations for a constant reference r and
# constant disturbance d, checks reference feasibility, and simulates a
# stabilizing deviation-feedback law around the computed equilibrium.
#
# Required libraries:
#   pip install numpy scipy matplotlib

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def equilibrium_for_constant_reference(A, B, C, D, E, F, r, d):
    """Solve the constant tracking equilibrium equations.

    At steady state, x_dot = 0 and e = y - r = 0, hence

        A x_bar + B u_bar + E d = 0
        C x_bar + D u_bar + F d = r

    The stacked equation is solved in the least-squares sense so that
    underactuated or incompatible references can be detected by the residual.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    D = np.asarray(D, dtype=float)
    E = np.asarray(E, dtype=float)
    F = np.asarray(F, dtype=float)
    r = np.asarray(r, dtype=float).reshape((-1, 1))
    d = np.asarray(d, dtype=float).reshape((-1, 1))

    M = np.block([[A, B],
                  [C, D]])
    rhs = np.vstack((-E @ d, r - F @ d))

    theta, *_ = np.linalg.lstsq(M, rhs, rcond=None)
    residual = M @ theta - rhs

    n = A.shape[0]
    x_bar = theta[:n]
    u_bar = theta[n:]
    return x_bar, u_bar, residual, M, rhs


def simulate_tracking_example():
    # Second-order plant: position output, force input, matched constant disturbance.
    A = np.array([[0.0, 1.0],
                  [-2.0, -0.8]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])
    E = np.array([[0.0],
                  [1.0]])
    F = np.array([[0.0]])

    r = np.array([1.0])      # constant reference position
    d = np.array([0.2])      # constant disturbance acceleration

    x_bar, u_bar, residual, M, rhs = equilibrium_for_constant_reference(
        A, B, C, D, E, F, r, d
    )

    print("Stacked equilibrium matrix [A B; C D]:")
    print(M)
    print("\nRight-hand side [-E d; r - F d]:")
    print(rhs)
    print("\nComputed x_bar:")
    print(x_bar)
    print("Computed u_bar:")
    print(u_bar)
    print("Equilibrium residual norm:", np.linalg.norm(residual))

    # The gain K is not designed in this lesson. It is used only to close
    # the loop around the formulated tracking equilibrium.
    K = np.array([[4.0, 2.2]])

    def closed_loop(t, x_flat):
        x = x_flat.reshape((-1, 1))
        u = u_bar - K @ (x - x_bar)
        x_dot = A @ x + B @ u + E @ d.reshape((-1, 1))
        return x_dot.ravel()

    t_span = (0.0, 8.0)
    t_eval = np.linspace(t_span[0], t_span[1], 801)
    x0 = np.array([0.0, 0.0])

    sol = solve_ivp(closed_loop, t_span, x0, t_eval=t_eval, rtol=1e-9, atol=1e-11)
    y = (C @ sol.y).ravel()
    e = y - r[0]

    print("\nFinal output y(T):", y[-1])
    print("Final tracking error e(T):", e[-1])

    plt.figure()
    plt.plot(sol.t, y, label="y(t)")
    plt.plot(sol.t, r[0] * np.ones_like(sol.t), "--", label="r")
    plt.plot(sol.t, e, label="e(t) = y(t) - r")
    plt.xlabel("time [s]")
    plt.ylabel("signals")
    plt.grid(True)
    plt.legend()
    plt.title("Chapter 27 Lesson 1: Tracking formulation simulation")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    simulate_tracking_example()
