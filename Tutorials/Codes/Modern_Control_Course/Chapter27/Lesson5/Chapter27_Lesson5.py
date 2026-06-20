"""
Chapter27_Lesson5.py

Case studies for reference tracking and disturbance rejection in state space.

Plant:
    x_dot = A x + B u + E d
    y     = C x

Case 1: state feedback plus static reference prefilter
    u = -K x + Nbar r

Case 2: integral servo for constant reference and constant disturbance
    z_dot = r - y
    u     = -Kx x + Ki z

Requires:
    numpy, scipy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import place_poles


def feedforward_gain(A: np.ndarray, B: np.ndarray, C: np.ndarray, K: np.ndarray) -> float:
    """Return Nbar so that y_ss = r for u = -Kx + Nbar r."""
    Acl = A - B @ K
    dc_gain = C @ np.linalg.solve(Acl, B)
    return float(-1.0 / dc_gain)


def simulate_feedforward_case(A, B, E, C, K, nbar, r_value, d_value, t_final=8.0):
    """Simulate x_dot = (A-BK)x + B*Nbar*r + E*d."""
    Acl = A - B @ K

    def rhs(t, x):
        return Acl @ x + (B.flatten() * nbar * r_value) + (E.flatten() * d_value)

    sol = solve_ivp(rhs, [0.0, t_final], np.zeros(A.shape[0]), max_step=0.01)
    y = (C @ sol.y).flatten()
    u = (-K @ sol.y + nbar * r_value).flatten()
    return sol.t, sol.y, y, u


def simulate_integral_case(A, B, E, C, Kx, Ki, r_value, d_value, t_final=8.0):
    """Simulate augmented system with z_dot = r - Cx and u = -Kx*x + Ki*z."""
    n = A.shape[0]

    def rhs(t, xa):
        x = xa[:n]
        z = xa[n]
        y = float(C @ x)
        u = float(-Kx @ x + Ki * z)
        xdot = A @ x + B.flatten() * u + E.flatten() * d_value
        zdot = r_value - y
        return np.r_[xdot, zdot]

    sol = solve_ivp(rhs, [0.0, t_final], np.zeros(n + 1), max_step=0.01)
    x = sol.y[:n, :]
    z = sol.y[n, :]
    y = (C @ x).flatten()
    u = (-Kx @ x + Ki * z).flatten()
    return sol.t, x, z, y, u


def main():
    # Mass-spring-damper plant: m*q_ddot + b*q_dot + k*q = u + d
    m = 1.0
    b = 0.6
    k = 2.0

    A = np.array([[0.0, 1.0],
                  [-k / m, -b / m]])
    B = np.array([[0.0],
                  [1.0 / m]])
    E = B.copy()  # matched force disturbance
    C = np.array([[1.0, 0.0]])

    # Feedforward-only state feedback design.
    desired_poles_ff = np.array([-2.0, -3.0])
    K = place_poles(A, B, desired_poles_ff).gain_matrix
    nbar = feedforward_gain(A, B, C, K)

    # Integral-servo design. Desired poles: two dominant poles and one faster integrator pole.
    A_aug = np.block([
        [A, np.zeros((2, 1))],
        [-C, np.zeros((1, 1))]
    ])
    B_aug = np.vstack([B, [[0.0]]])
    desired_poles_int = np.array([-2.0, -3.0, -5.0])
    K_aug = place_poles(A_aug, B_aug, desired_poles_int).gain_matrix
    Kx = K_aug[:, :2]
    Ki = -K_aug[0, 2]  # because place_poles assumes u = -[Kx Kz][x;z]; here u=-Kx*x+Ki*z

    r_value = 1.0
    d_value = 0.4

    t0, x0, y0, u0 = simulate_feedforward_case(A, B, E, C, K, nbar, r_value, 0.0)
    t1, x1, y1, u1 = simulate_feedforward_case(A, B, E, C, K, nbar, r_value, d_value)
    t2, x2, z2, y2, u2 = simulate_integral_case(A, B, E, C, Kx, Ki, r_value, d_value)

    print("Feedforward K:", K)
    print("Feedforward Nbar:", nbar)
    print("Integral Kx:", Kx)
    print("Integral Ki:", Ki)
    print("Final y, feedforward no disturbance:", y0[-1])
    print("Final y, feedforward with disturbance:", y1[-1])
    print("Final y, integral with disturbance:", y2[-1])

    plt.figure()
    plt.plot(t0, y0, label="FF, d=0")
    plt.plot(t1, y1, label="FF, d=0.4")
    plt.plot(t2, y2, label="Integral, d=0.4")
    plt.axhline(r_value, linestyle="--", label="reference")
    plt.xlabel("time [s]")
    plt.ylabel("output y")
    plt.title("Reference Tracking and Disturbance Rejection")
    plt.legend()
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.plot(t1, u1, label="FF, d=0.4")
    plt.plot(t2, u2, label="Integral, d=0.4")
    plt.xlabel("time [s]")
    plt.ylabel("control input u")
    plt.title("Control Effort")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
