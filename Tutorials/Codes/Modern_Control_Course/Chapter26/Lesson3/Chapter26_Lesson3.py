"""
Chapter26_Lesson3.py
Design of state-feedback gains for an augmented system with integral action.

Required libraries:
    pip install numpy scipy matplotlib
Optional control-system library:
    pip install control
"""

import numpy as np
from scipy.signal import place_poles
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def build_augmented_system(A, B, C, D=None):
    """Build A_aug and B_aug for z_dot = r - y, y = Cx + Du."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    n = A.shape[0]
    p = C.shape[0]
    if D is None:
        D = np.zeros((p, B.shape[1]))
    D = np.asarray(D, dtype=float)

    A_aug = np.block([
        [A, np.zeros((n, p))],
        [-C, np.zeros((p, p))]
    ])
    B_aug = np.vstack((B, -D))
    E_aug = np.vstack((np.zeros((n, p)), np.eye(p)))
    return A_aug, B_aug, E_aug


def controllability_matrix(A, B):
    """Kalman controllability matrix [B AB ... A^(n-1)B]."""
    n = A.shape[0]
    return np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(n)])


def main():
    # Mass-spring-damper example:
    # x1 = position, x2 = velocity, u = force, y = position.
    A = np.array([[0.0, 1.0], [-2.0, -0.6]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    A_aug, B_aug, E_aug = build_augmented_system(A, B, C, D)

    Co = controllability_matrix(A_aug, B_aug)
    rank_co = np.linalg.matrix_rank(Co)
    print("Augmented controllability rank:", rank_co, "of", A_aug.shape[0])

    desired_poles = np.array([-2.0, -2.5, -3.0])
    placed = place_poles(A_aug, B_aug, desired_poles)
    K_aug = placed.gain_matrix
    Kx = K_aug[:, :A.shape[0]]
    Ki = K_aug[:, A.shape[0]:]

    print("K_aug =", K_aug)
    print("Kx =", Kx)
    print("Ki =", Ki)
    print("Closed-loop eigenvalues =", np.linalg.eigvals(A_aug - B_aug @ K_aug))

    r = np.array([1.0])

    def closed_loop_ode(t, xa):
        xa = xa.reshape(-1, 1)
        u = -K_aug @ xa
        dxa = (A_aug - B_aug @ K_aug) @ xa + E_aug @ r.reshape(-1, 1)
        return dxa.ravel()

    t_eval = np.linspace(0.0, 8.0, 800)
    sol = solve_ivp(closed_loop_ode, (t_eval[0], t_eval[-1]), np.zeros(3), t_eval=t_eval)

    x = sol.y[:2, :]
    z = sol.y[2, :]
    y = (C @ x).ravel()
    u = (-K_aug @ sol.y).ravel()
    e = r[0] - y

    print("Final output:", y[-1])
    print("Final tracking error:", e[-1])
    print("Final integrator state:", z[-1])

    plt.figure()
    plt.plot(sol.t, y, label="output y")
    plt.plot(sol.t, np.ones_like(sol.t) * r[0], "--", label="reference r")
    plt.xlabel("time [s]")
    plt.ylabel("position")
    plt.grid(True)
    plt.legend()
    plt.title("State feedback with integral action")
    plt.show()

    plt.figure()
    plt.plot(sol.t, u, label="control input u")
    plt.xlabel("time [s]")
    plt.ylabel("force")
    plt.grid(True)
    plt.legend()
    plt.title("Control effort")
    plt.show()


if __name__ == "__main__":
    main()
