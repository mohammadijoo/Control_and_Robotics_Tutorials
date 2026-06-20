# Chapter30_Lesson4.py
# Integrated case study: mass-spring-damper model to state-feedback controller
# Libraries: NumPy for linear algebra. SciPy is optional for comparison with place_poles.

import math
import numpy as np


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return [B, AB] for a two-state SISO model."""
    return np.column_stack((B, A @ B))


def state_feedback_from_specs(m: float, b: float, k: float, zeta: float, settling_time: float):
    """Compute K = [k1, k2] by matching the desired second-order polynomial."""
    omega_n = 4.0 / (zeta * settling_time)
    k1 = m * omega_n**2 - k
    k2 = 2.0 * zeta * omega_n * m - b
    return np.array([[k1, k2]], dtype=float), omega_n


def feedforward_gain(A: np.ndarray, B: np.ndarray, C: np.ndarray, K: np.ndarray) -> float:
    """Nbar = -1/(C (A-BK)^(-1) B), valid when the scalar denominator is nonzero."""
    Acl = A - B @ K
    denom = C @ np.linalg.solve(Acl, B)
    return float(-1.0 / denom[0, 0])


def rk4_step(f, t, x, h):
    k1 = f(t, x)
    k2 = f(t + 0.5 * h, x + 0.5 * h * k1)
    k3 = f(t + 0.5 * h, x + 0.5 * h * k2)
    k4 = f(t + h, x + h * k3)
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_closed_loop(A, B, C, K, nbar, r=1.0, t_final=6.0, dt=0.002):
    Acl = A - B @ K
    Bcl = B * nbar

    def f(t, x):
        return Acl @ x + Bcl.flatten() * r

    steps = int(t_final / dt)
    t = np.zeros(steps + 1)
    x = np.zeros((steps + 1, 2))
    y = np.zeros(steps + 1)
    u = np.zeros(steps + 1)
    for i in range(steps):
        t[i + 1] = t[i] + dt
        y[i] = float(C @ x[i])
        u[i] = float(-K @ x[i].reshape(-1, 1) + nbar * r)
        x[i + 1] = rk4_step(f, t[i], x[i], dt)
    y[-1] = float(C @ x[-1])
    u[-1] = float(-K @ x[-1].reshape(-1, 1) + nbar * r)
    return t, x, y, u


def main():
    m = 1.2       # kg
    b = 0.8       # N*s/m
    k = 3.0       # N/m
    zeta = 0.70
    settling_time = 2.0

    A = np.array([[0.0, 1.0], [-k / m, -b / m]])
    B = np.array([[0.0], [1.0 / m]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    Wc = controllability_matrix(A, B)
    print("A =\n", A)
    print("B =\n", B)
    print("Controllability matrix =\n", Wc)
    print("rank(Wc) =", np.linalg.matrix_rank(Wc))

    K, omega_n = state_feedback_from_specs(m, b, k, zeta, settling_time)
    nbar = feedforward_gain(A, B, C, K)
    Acl = A - B @ K

    print("omega_n =", omega_n)
    print("K =", K)
    print("Nbar =", nbar)
    print("closed-loop poles =", np.linalg.eigvals(Acl))

    t, x, y, u = simulate_closed_loop(A, B, C, K, nbar, r=1.0)
    print("final output y(t_final) =", y[-1])
    print("maximum absolute control input =", np.max(np.abs(u)))

    # Save a small CSV for plotting in any external tool.
    data = np.column_stack((t, x[:, 0], x[:, 1], y, u))
    np.savetxt(
        "Chapter30_Lesson4_response.csv",
        data,
        delimiter=",",
        header="t,position,velocity,output,control",
        comments="",
    )


if __name__ == "__main__":
    main()
