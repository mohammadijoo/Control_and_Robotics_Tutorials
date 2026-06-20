# Chapter30_Lesson5.py
"""
Bridge lab for Modern Control, Chapter 30 Lesson 5.

Topic:
    Connections among pole placement, LQR, robustness checks,
    finite-horizon MPC, MIMO/state-space thinking, and observer design.

Dependencies:
    numpy, scipy
Optional:
    matplotlib for plots if you extend the script.

Run:
    python Chapter30_Lesson5.py
"""
from __future__ import annotations

import numpy as np
from numpy.linalg import inv
from scipy.linalg import solve_continuous_are, eigvals
from scipy.signal import place_poles, cont2discrete


def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray):
    """Continuous-time infinite-horizon LQR."""
    P = solve_continuous_are(A, B, Q, R)
    K = inv(R) @ B.T @ P
    closed_loop_poles = eigvals(A - B @ K)
    return K, P, closed_loop_poles


def observer_gain(A: np.ndarray, C: np.ndarray, desired_poles):
    """Luenberger observer gain by dual pole placement."""
    placed = place_poles(A.T, C.T, desired_poles)
    L = placed.gain_matrix.T
    return L, eigvals(A - L @ C)


def pole_placement_gain(A: np.ndarray, B: np.ndarray, desired_poles):
    """State-feedback pole placement."""
    placed = place_poles(A, B, desired_poles)
    K = placed.gain_matrix
    return K, eigvals(A - B @ K)


def robustness_scan(A: np.ndarray, B: np.ndarray, K: np.ndarray, E: np.ndarray, deltas):
    """
    Scan a scalar structured uncertainty A_delta = A + delta E.
    This is not a proof of robust stability; it is a numerical warning test.
    """
    rows = []
    for delta in deltas:
        lam = eigvals(A + delta * E - B @ K)
        rows.append((float(delta), float(np.max(np.real(lam)))))
    return rows


def finite_horizon_lqr_gain(Ad: np.ndarray, Bd: np.ndarray, Q: np.ndarray,
                            R: np.ndarray, Qf: np.ndarray, horizon: int):
    """
    Unconstrained finite-horizon MPC/LQR.
    The first control law is u_0 = -K_0 x_0.
    """
    P = Qf.copy()
    gains = []
    for _ in range(horizon, 0, -1):
        S = R + Bd.T @ P @ Bd
        K = inv(S) @ (Bd.T @ P @ Ad)
        gains.append(K)
        P = Q + Ad.T @ P @ (Ad - Bd @ K)
    gains.reverse()
    return gains[0], gains, P


def simulate_output_feedback(A: np.ndarray, B: np.ndarray, C: np.ndarray,
                             K: np.ndarray, L: np.ndarray,
                             x0: np.ndarray, xhat0: np.ndarray,
                             dt: float = 0.001, tf: float = 4.0):
    """
    Euler simulation of observer-based state feedback:
        u = -K xhat
        xhat_dot = A xhat + B u + L(y - C xhat)
    """
    steps = int(tf / dt)
    x = x0.astype(float).copy()
    xhat = xhat0.astype(float).copy()
    history = []
    for k in range(steps + 1):
        t = k * dt
        y = C @ x
        u = -K @ xhat
        history.append((t, x.copy(), xhat.copy(), float(u[0, 0])))
        xdot = A @ x + B @ u
        xhat_dot = A @ xhat + B @ u + L @ (y - C @ xhat)
        x += dt * xdot
        xhat += dt * xhat_dot
    return history


def main():
    # Double-integrator plant: position and velocity states.
    A = np.array([[0.0, 1.0],
                  [0.0, 0.0]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])

    # 1) Pole placement: choose closed-loop modes directly.
    K_pp, pp_poles = pole_placement_gain(A, B, [-2.0, -3.0])

    # 2) LQR: choose a quadratic performance index instead of explicit poles.
    Q = np.diag([10.0, 1.0])
    R = np.array([[0.25]])
    K_lqr, P, lqr_poles = lqr_gain(A, B, Q, R)

    # 3) Observer: estimate states from output y = position.
    L, observer_poles = observer_gain(A, C, [-8.0, -9.0])

    # 4) Robustness preview: perturb the spring-like term A[1,0].
    E = np.array([[0.0, 0.0],
                  [1.0, 0.0]])
    deltas = np.linspace(-4.0, 4.0, 17)
    scan = robustness_scan(A, B, K_lqr, E, deltas)

    # 5) MPC preview: discretize and compute first finite-horizon gain.
    Ts = 0.05
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, np.zeros((1, 1))), Ts)
    K_mpc0, gains, _ = finite_horizon_lqr_gain(
        Ad, Bd, Q=np.diag([10.0, 1.0]), R=np.array([[0.25]]),
        Qf=P, horizon=25
    )

    # 6) Separation principle simulation.
    history = simulate_output_feedback(
        A, B, C, K_lqr, L,
        x0=np.array([[1.0], [0.0]]),
        xhat0=np.array([[0.0], [0.0]]),
        dt=0.002,
        tf=4.0
    )

    print("Pole-placement K:", K_pp)
    print("Pole-placement closed-loop poles:", pp_poles)
    print("\nLQR K:", K_lqr)
    print("LQR Riccati P:\n", P)
    print("LQR closed-loop poles:", lqr_poles)
    print("\nObserver L:\n", L)
    print("Observer poles:", observer_poles)
    print("\nFinite-horizon MPC first gain K0:", K_mpc0)
    print("\nRobustness scan: delta, max real eigenvalue")
    for delta, max_real in scan:
        stable = "stable" if max_real < 0.0 else "unstable"
        print(f"{delta:+.2f}  {max_real:+.4f}  {stable}")

    final_t, final_x, final_xhat, final_u = history[-1]
    print("\nOutput-feedback simulation final state:")
    print("t =", final_t)
    print("x =", final_x.ravel())
    print("xhat =", final_xhat.ravel())
    print("u =", final_u)


if __name__ == "__main__":
    main()
