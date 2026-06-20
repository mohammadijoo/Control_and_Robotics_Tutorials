"""
Chapter29_Lesson5.py

Engineering example for slowly time-varying linear systems.
System: mass-spring-damper oscillator with slowly drifting mass, damping, and stiffness.

State:
    x1 = displacement
    x2 = velocity

Dynamics:
    x_dot = A(t) x + B(t) u(t)

Required libraries:
    numpy
    scipy
    matplotlib
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm
import matplotlib.pyplot as plt


EPSILON = 0.03


def mass(t):
    return 1.0 + 0.12 * np.sin(EPSILON * t)


def damping(t):
    return 0.22 + 0.04 * np.cos(EPSILON * t)


def stiffness(t):
    return 2.0 + 0.30 * np.sin(0.5 * EPSILON * t)


def input_force(t):
    return 0.2 * np.sin(0.7 * t)


def A_matrix(t):
    m = mass(t)
    c = damping(t)
    k = stiffness(t)
    return np.array([[0.0, 1.0], [-k / m, -c / m]], dtype=float)


def B_matrix(t):
    return np.array([0.0, 1.0 / mass(t)], dtype=float)


def rhs(t, x):
    return A_matrix(t).dot(x) + B_matrix(t) * input_force(t)


def frozen_step(A, b, u_value, x, dt):
    """
    Exact step for frozen dynamics over [t, t + dt]:
        x_dot = A x + b u_value
    The input is frozen during the step.
    """
    n = A.shape[0]
    aug = np.zeros((n + 1, n + 1))
    aug[:n, :n] = A
    aug[:n, n] = b * u_value
    phi_aug = expm(aug * dt)
    return phi_aug[:n, :n].dot(x) + phi_aug[:n, n]


def simulate_frozen_grid(t_grid, x0):
    x = np.zeros((len(t_grid), 2))
    x[0] = x0
    for i in range(len(t_grid) - 1):
        t = t_grid[i]
        dt = t_grid[i + 1] - t_grid[i]
        x[i + 1] = frozen_step(A_matrix(t), B_matrix(t), input_force(t), x[i], dt)
    return x


def slow_variation_index(t_grid):
    """
    Compute a numerical index:
        eta(t) = ||dA/dt||_F / alpha(t)^2
    where alpha(t) is the frozen-time decay margin:
        alpha(t) = -max Re(lambda_i(A(t)))
    A small eta suggests a slowly varying regime relative to the local decay rate.
    """
    values = []
    for t in t_grid:
        h = 1e-3
        dA = (A_matrix(t + h) - A_matrix(t - h)) / (2.0 * h)
        eigvals = np.linalg.eigvals(A_matrix(t))
        alpha = -np.max(np.real(eigvals))
        eta = np.linalg.norm(dA, ord="fro") / max(alpha * alpha, 1e-12)
        values.append((t, alpha, eta))
    return np.array(values)


def main():
    t0, tf = 0.0, 160.0
    x0 = np.array([1.0, 0.0])
    t_eval = np.linspace(t0, tf, 2001)

    sol = solve_ivp(rhs, (t0, tf), x0, t_eval=t_eval, rtol=1e-9, atol=1e-11)
    frozen = simulate_frozen_grid(t_eval, x0)
    svi = slow_variation_index(t_eval[::100])

    print("Final continuous LTV state:", sol.y[:, -1])
    print("Final frozen-step state:", frozen[-1])
    print("Maximum absolute frozen-step error:", np.max(np.abs(sol.y.T - frozen)))
    print("Maximum slow-variation index eta:", np.max(svi[:, 2]))

    np.savetxt(
        "Chapter29_Lesson5_python_output.csv",
        np.column_stack((t_eval, sol.y.T, frozen)),
        delimiter=",",
        header="t,x1_ltv,x2_ltv,x1_frozen,x2_frozen",
        comments="",
    )

    plt.figure()
    plt.plot(t_eval, sol.y[0], label="LTV displacement")
    plt.plot(t_eval, frozen[:, 0], "--", label="frozen-step displacement")
    plt.xlabel("time")
    plt.ylabel("displacement")
    plt.title("Slowly Time-Varying Mass-Spring-Damper System")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("Chapter29_Lesson5_python_plot.png", dpi=180)
    plt.show()


if __name__ == "__main__":
    main()
