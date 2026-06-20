"""
Chapter27_Lesson4.py
State-space design for disturbance rejection.

Benchmark plant:
    x_dot = A x + B u + E d
    y     = C x

The script compares:
1. state feedback only,
2. measured-disturbance feedforward from regulator equations,
3. unmeasured constant-disturbance rejection using integral action.

Required libraries:
    numpy, scipy, matplotlib
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import place_poles
import matplotlib.pyplot as plt


A = np.array([[0.0, 1.0], [-2.0, -0.6]])
B = np.array([[0.0], [1.0]])
E = np.array([[0.0], [1.0]])
C = np.array([[1.0, 0.0]])

d0 = 0.5
x0 = np.array([0.4, 0.0])
t_span = (0.0, 10.0)
t_eval = np.linspace(t_span[0], t_span[1], 1000)


def simulate_state_feedback_only():
    """Closed loop u = -Kx. Constant disturbance generally leaves offset."""
    K = place_poles(A, B, [-2.0, -3.0]).gain_matrix

    def rhs(_t, x):
        u = (-K @ x.reshape(-1, 1)).item()
        return (A @ x.reshape(-1, 1) + B * u + E * d0).ravel()

    sol = solve_ivp(rhs, t_span, x0, t_eval=t_eval, rtol=1e-9, atol=1e-11)
    y = (C @ sol.y).ravel()
    return sol.t, y, K


def measured_disturbance_feedforward():
    """Solve regulator equations for known constant d."""
    K = place_poles(A, B, [-2.0, -3.0]).gain_matrix

    block = np.block([[A - B @ K, B], [C, np.zeros((1, 1))]])
    rhs_reg = np.vstack([-E, [[0.0]]])
    solution = np.linalg.solve(block, rhs_reg)
    Pi = solution[:2, :]
    Gamma = solution[2:, :]

    def rhs(_t, x):
        # u = -Kx + Gamma d, where Gamma is obtained from regulator equations.
        u = (-K @ x.reshape(-1, 1) + Gamma * d0).item()
        return (A @ x.reshape(-1, 1) + B * u + E * d0).ravel()

    sol = solve_ivp(rhs, t_span, x0, t_eval=t_eval, rtol=1e-9, atol=1e-11)
    y = (C @ sol.y).ravel()
    return sol.t, y, K, Pi, Gamma


def integral_disturbance_rejection():
    """Unmeasured constant disturbance rejection with xi_dot = y."""
    A_aug = np.block([[A, np.zeros((2, 1))], [C, np.zeros((1, 1))]])
    B_aug = np.vstack([B, [[0.0]]])
    E_aug = np.vstack([E, [[0.0]]])

    K_aug = place_poles(A_aug, B_aug, [-2.0, -2.5, -3.0]).gain_matrix
    K = K_aug[:, :2]
    Ki = K_aug[:, 2:]

    def rhs(_t, xa):
        x = xa[:2].reshape(-1, 1)
        xi = np.array([[xa[2]]])
        u = (-K @ x - Ki @ xi).item()
        dx = A @ x + B * u + E * d0
        dxi = C @ x
        return np.vstack([dx, dxi]).ravel()

    sol = solve_ivp(rhs, t_span, np.array([x0[0], x0[1], 0.0]),
                    t_eval=t_eval, rtol=1e-9, atol=1e-11)
    y = (C @ sol.y[:2, :]).ravel()
    return sol.t, y, K, Ki, A_aug, B_aug, E_aug


if __name__ == "__main__":
    t1, y1, K1 = simulate_state_feedback_only()
    t2, y2, K2, Pi, Gamma = measured_disturbance_feedforward()
    t3, y3, K3, Ki, A_aug, B_aug, E_aug = integral_disturbance_rejection()

    print("State feedback K:", K1)
    print("Measured-disturbance regulator Pi:\n", Pi)
    print("Measured-disturbance feedforward Gamma:", Gamma)
    print("Integral controller K:", K3)
    print("Integral gain Ki:", Ki)
    print("Final outputs:")
    print("  state feedback only        =", y1[-1])
    print("  measured feedforward       =", y2[-1])
    print("  integral disturbance reject=", y3[-1])

    plt.figure(figsize=(8, 4.8))
    plt.plot(t1, y1, label="state feedback only")
    plt.plot(t2, y2, label="measured feedforward")
    plt.plot(t3, y3, label="integral action")
    plt.axhline(0.0, linestyle="--", linewidth=1)
    plt.xlabel("time [s]")
    plt.ylabel("controlled output y")
    plt.title("Disturbance rejection in state space")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
