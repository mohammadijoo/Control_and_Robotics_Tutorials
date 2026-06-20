"""
Chapter26_Lesson4.py
State-feedback with integral action for step and ramp reference inputs.

Required libraries:
    pip install numpy scipy matplotlib
Optional control library for independent verification:
    pip install control
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import place_poles
import matplotlib.pyplot as plt


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    Apow = np.eye(n)
    for _ in range(1, n):
        Apow = Apow @ A
        blocks.append(Apow @ B)
    return np.hstack(blocks)


def rank_report(name: str, A: np.ndarray, B: np.ndarray) -> None:
    Ctrb = controllability_matrix(A, B)
    print(f"{name}: rank(C) = {np.linalg.matrix_rank(Ctrb)} / {A.shape[0]}")


# SISO plant: x_dot = A x + B u, y = C x
A = np.array([[0.0, 1.0], [-2.0, -3.0]])
B = np.array([[0.0], [1.0]])
C = np.array([[1.0, 0.0]])

# ---------------------------------------------------------------------------
# 1) Step-reference tracking: one error integrator
# eta_dot = r - y = r - C x
# z = [x1, x2, eta]^T
# z_dot = A_step z + B_step u + E_step r
# u = -K_step z
# ---------------------------------------------------------------------------
A_step = np.block([
    [A, np.zeros((2, 1))],
    [-C, np.zeros((1, 1))]
])
B_step = np.vstack([B, [[0.0]]])
E_step = np.array([[0.0], [0.0], [1.0]])
rank_report("Step-augmented system", A_step, B_step)

poles_step = np.array([-4.0, -5.0, -6.0])
K_step = place_poles(A_step, B_step, poles_step).gain_matrix
print("K_step =", K_step)


def step_closed_loop(t: float, z: np.ndarray) -> np.ndarray:
    r = 1.0
    u = (-K_step @ z.reshape(-1, 1)).item()
    dz = A_step @ z.reshape(-1, 1) + B_step * u + E_step * r
    return dz.ravel()


# ---------------------------------------------------------------------------
# 2) Ramp-reference tracking: two error-integrator states
# eta1_dot = r - y, eta2_dot = eta1
# z = [x1, x2, eta1, eta2]^T
# u = -K_ramp z
# ---------------------------------------------------------------------------
A_ramp = np.block([
    [A, np.zeros((2, 2))],
    [-C, np.zeros((1, 2))],
    [np.zeros((1, 2)), np.array([[1.0, 0.0]])]
])
B_ramp = np.vstack([B, [[0.0], [0.0]]])
E_ramp = np.array([[0.0], [0.0], [1.0], [0.0]])
rank_report("Ramp-augmented system", A_ramp, B_ramp)

poles_ramp = np.array([-3.0, -4.0, -5.0, -6.0])
K_ramp = place_poles(A_ramp, B_ramp, poles_ramp).gain_matrix
print("K_ramp =", K_ramp)


def ramp_closed_loop(t: float, z: np.ndarray) -> np.ndarray:
    r = t
    u = (-K_ramp @ z.reshape(-1, 1)).item()
    dz = A_ramp @ z.reshape(-1, 1) + B_ramp * u + E_ramp * r
    return dz.ravel()


def simulate_and_plot() -> None:
    t_eval = np.linspace(0.0, 8.0, 801)

    sol_step = solve_ivp(
        step_closed_loop,
        (t_eval[0], t_eval[-1]),
        y0=np.zeros(3),
        t_eval=t_eval,
        rtol=1e-9,
        atol=1e-11,
    )
    y_step = (C @ sol_step.y[:2, :]).ravel()
    r_step = np.ones_like(t_eval)

    sol_ramp = solve_ivp(
        ramp_closed_loop,
        (t_eval[0], t_eval[-1]),
        y0=np.zeros(4),
        t_eval=t_eval,
        rtol=1e-9,
        atol=1e-11,
    )
    y_ramp = (C @ sol_ramp.y[:2, :]).ravel()
    r_ramp = t_eval

    print("Final step error:", r_step[-1] - y_step[-1])
    print("Final ramp error:", r_ramp[-1] - y_ramp[-1])

    plt.figure()
    plt.plot(t_eval, r_step, "--", label="step reference")
    plt.plot(t_eval, y_step, label="output")
    plt.xlabel("time [s]")
    plt.ylabel("y(t)")
    plt.title("Step tracking with one error integrator")
    plt.grid(True)
    plt.legend()

    plt.figure()
    plt.plot(t_eval, r_ramp, "--", label="ramp reference")
    plt.plot(t_eval, y_ramp, label="output")
    plt.xlabel("time [s]")
    plt.ylabel("y(t)")
    plt.title("Ramp tracking with two error integrators")
    plt.grid(True)
    plt.legend()

    plt.show()


if __name__ == "__main__":
    simulate_and_plot()
