# Chapter27_Lesson3.py
# Internal Model Principle for step tracking in a continuous-time state-space plant.
# Required packages: numpy, scipy

import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import place_poles


def build_augmented_step_servo():
    """Plant:
        x_dot = A x + B u
        y     = C x
       Internal model for a constant reference:
        eta_dot = r - y
       Control:
        u = -Kx x + Ki eta
    """
    A = np.array([[0.0, 1.0],
                  [-2.0, -0.4]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])

    # Augmented state xa = [x1, x2, eta]^T, eta_dot = r - Cx.
    A_aug = np.block([
        [A, np.zeros((2, 1))],
        [-C, np.zeros((1, 1))]
    ])
    B_aug = np.vstack([B, [[0.0]]])

    desired_poles = np.array([-2.0 + 2.0j, -2.0 - 2.0j, -5.0])
    placed = place_poles(A_aug, B_aug, desired_poles)
    K_aug = placed.gain_matrix

    # Since u = -K_aug * [x; eta], and the integrator state is eta_dot = -Cx + r,
    # the third gain is negative for the convention u = -Kx + Ki eta.
    Kx = K_aug[:, :2]
    Ki = -K_aug[:, 2:3]
    return A, B, C, Kx, Ki, A_aug, B_aug


def simulate_step_response(t_final=8.0, reference=1.0):
    A, B, C, Kx, Ki, _, _ = build_augmented_step_servo()

    def closed_loop_rhs(t, xa):
        x = xa[:2].reshape(2, 1)
        eta = xa[2]
        y = float(C @ x)
        u = float(-Kx @ x + Ki * eta)
        xdot = A @ x + B * u
        etadot = reference - y
        return np.array([xdot[0, 0], xdot[1, 0], etadot])

    sol = solve_ivp(
        closed_loop_rhs,
        t_span=(0.0, t_final),
        y0=np.array([0.0, 0.0, 0.0]),
        max_step=0.01,
        rtol=1e-8,
        atol=1e-10,
    )

    x1 = sol.y[0, :]
    x2 = sol.y[1, :]
    eta = sol.y[2, :]
    y = x1
    u = -Kx[0, 0] * x1 - Kx[0, 1] * x2 + Ki[0, 0] * eta
    e = reference - y
    return sol.t, y, u, e, Kx, Ki


def solve_regulator_equations_for_constant_reference():
    """Solve the steady-state regulator equations for S = 0 and y_ref = w.
       A*pi + B*gamma = 0
       C*pi = 1
       This gives the steady-state x = pi*w and u = gamma*w.
    """
    A = np.array([[0.0, 1.0],
                  [-2.0, -0.4]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])

    M = np.block([
        [A, B],
        [C, np.zeros((1, 1))]
    ])
    rhs = np.array([[0.0], [0.0], [1.0]])
    sol = np.linalg.solve(M, rhs)
    pi = sol[:2, :]
    gamma = sol[2:, :]
    return pi, gamma


if __name__ == "__main__":
    t, y, u, e, Kx, Ki = simulate_step_response()
    pi, gamma = solve_regulator_equations_for_constant_reference()

    print("Kx =", Kx)
    print("Ki =", Ki)
    print("Regulator equation Pi =", pi.ravel())
    print("Regulator equation Gamma =", gamma.ravel())
    print("Final output y(T) =", y[-1])
    print("Final tracking error e(T) =", e[-1])
    print("Peak absolute control =", np.max(np.abs(u)))
