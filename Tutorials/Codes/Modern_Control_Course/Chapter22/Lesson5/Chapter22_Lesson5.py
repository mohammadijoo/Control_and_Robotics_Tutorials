"""
Chapter22_Lesson5.py

Practical constraints and limitations of state feedback:
- ideal full-state feedback
- actuator amplitude saturation
- actuator rate limiting
- measurement noise amplification through K
- Lyapunov ellipsoid estimate for the no-saturation region

Dependencies:
    pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt


def solve_continuous_lyapunov_by_kron(A: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """
    Solve A.T P + P A = -Q using vectorization:
        vec(A.T P + P A) = (I kron A.T + A.T kron I) vec(P)
    This avoids requiring scipy.
    """
    n = A.shape[0]
    M = np.kron(np.eye(n), A.T) + np.kron(A.T, np.eye(n))
    p = np.linalg.solve(M, -Q.reshape(-1, order="F"))
    return p.reshape((n, n), order="F")


def saturate(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def rate_limit(new_value: float, old_value: float, max_rate: float, dt: float) -> float:
    step = max_rate * dt
    return saturate(new_value, old_value - step, old_value + step)


def simulate(constrained: bool, noisy: bool):
    # Double integrator: position and velocity.
    A = np.array([[0.0, 1.0],
                  [0.0, 0.0]])
    B = np.array([[0.0],
                  [1.0]])

    # Pole placement for desired poles -2 and -3 gives K = [6, 5].
    K = np.array([[6.0, 5.0]])
    umax = 1.0
    max_rate = 8.0
    noise_std = np.array([0.02, 0.03])

    dt = 0.001
    tf = 6.0
    t = np.arange(0.0, tf + dt, dt)
    x = np.zeros((len(t), 2))
    u = np.zeros(len(t))
    x[0] = np.array([1.2, 0.0])

    rng = np.random.default_rng(4)
    for k in range(len(t) - 1):
        x_meas = x[k].copy()
        if noisy:
            x_meas += rng.normal(0.0, noise_std)

        u_cmd = float(-K @ x_meas)
        if constrained:
            u_sat = saturate(u_cmd, -umax, umax)
            u[k] = rate_limit(u_sat, u[k - 1] if k > 0 else 0.0, max_rate, dt)
        else:
            u[k] = u_cmd

        # Fourth-order Runge-Kutta for xdot = A x + B u, with u held constant.
        def f(xv):
            return A @ xv + (B[:, 0] * u[k])

        k1 = f(x[k])
        k2 = f(x[k] + 0.5 * dt * k1)
        k3 = f(x[k] + 0.5 * dt * k2)
        k4 = f(x[k] + dt * k3)
        x[k + 1] = x[k] + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    u[-1] = u[-2]
    return t, x, u, A, B, K


def lyapunov_no_saturation_radius(A: np.ndarray, B: np.ndarray, K: np.ndarray, umax: float):
    """
    For Acl = A - B K and Lyapunov ellipsoid E_rho = {x: x.T P x <= rho},
    saturation is certainly inactive if
        sqrt(rho * k P^{-1} k.T) <= umax.
    Thus rho <= umax^2 / (k P^{-1} k.T).
    """
    Acl = A - B @ K
    Q = np.eye(A.shape[0])
    P = solve_continuous_lyapunov_by_kron(Acl, Q)
    Pinv = np.linalg.inv(P)
    denom = float(K @ Pinv @ K.T)
    rho_max = umax * umax / denom
    return P, rho_max


if __name__ == "__main__":
    t1, x_ideal, u_ideal, A, B, K = simulate(constrained=False, noisy=False)
    t2, x_con, u_con, _, _, _ = simulate(constrained=True, noisy=False)
    t3, x_noise, u_noise, _, _, _ = simulate(constrained=True, noisy=True)

    P, rho = lyapunov_no_saturation_radius(A, B, K, umax=1.0)
    print("K =", K)
    print("Lyapunov P =")
    print(P)
    print("Guaranteed no-saturation ellipsoid radius rho <=", rho)

    plt.figure()
    plt.plot(t1, x_ideal[:, 0], label="ideal x1")
    plt.plot(t2, x_con[:, 0], label="constrained x1")
    plt.plot(t3, x_noise[:, 0], label="constrained+noise x1")
    plt.xlabel("time [s]")
    plt.ylabel("position state")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.figure()
    plt.plot(t1, u_ideal, label="ideal u")
    plt.plot(t2, u_con, label="saturated/rate-limited u")
    plt.plot(t3, u_noise, label="saturated/rate-limited/noisy u")
    plt.xlabel("time [s]")
    plt.ylabel("control input")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()
