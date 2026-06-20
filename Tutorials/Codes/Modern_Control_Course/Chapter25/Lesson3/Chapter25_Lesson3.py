"""
Chapter25_Lesson3.py

Trade-offs in state-feedback design:
speed of response vs. control effort vs. sensitivity.

Dependencies:
    pip install numpy scipy matplotlib

Optional extension:
    pip install control
"""

import numpy as np
from scipy.signal import place_poles
from scipy.linalg import solve_continuous_lyapunov
import matplotlib.pyplot as plt


def closed_loop_metrics(A, B, poles, x0):
    """Return K, closed-loop eigenvalues, effort Gramian, and J_u for u=-Kx."""
    result = place_poles(A, B, poles, method="YT")
    K = result.gain_matrix
    Ac = A - B @ K

    # P_u solves Ac.T P + P Ac + K.T K = 0.
    P_u = solve_continuous_lyapunov(Ac.T, -(K.T @ K))
    effort = (x0.T @ P_u @ x0).item()

    eigvals = np.linalg.eigvals(Ac)
    speed = -np.max(np.real(eigvals))
    k_norm = float(np.linalg.norm(K, 2))

    return K, Ac, eigvals, speed, k_norm, effort, P_u


def rk4_simulate(Ac, K, x0, tf=10.0, dt=0.002):
    """Simulate x_dot = Ac x and u = -Kx."""
    n_steps = int(tf / dt) + 1
    t = np.linspace(0.0, tf, n_steps)
    x = np.zeros((n_steps, len(x0)))
    u = np.zeros(n_steps)
    x[0, :] = x0.ravel()

    def f(xv):
        return Ac @ xv

    for k in range(n_steps - 1):
        xv = x[k, :]
        k1 = f(xv)
        k2 = f(xv + 0.5 * dt * k1)
        k3 = f(xv + 0.5 * dt * k2)
        k4 = f(xv + dt * k3)
        x[k + 1, :] = xv + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        u[k] = float(-(K @ x[k, :].reshape(-1, 1))[0, 0])
    u[-1] = float(-(K @ x[-1, :].reshape(-1, 1))[0, 0])
    return t, x, u


def monte_carlo_eigenvalue_sensitivity(A, B, K, samples=200, eps=0.02, seed=7):
    """Perturb A and B and measure the spread of closed-loop eigenvalues."""
    rng = np.random.default_rng(seed)
    eigs = []
    for _ in range(samples):
        dA = eps * rng.standard_normal(A.shape)
        dB = eps * rng.standard_normal(B.shape)
        Ac_perturbed = (A + dA) - (B + dB) @ K
        eigs.append(np.linalg.eigvals(Ac_perturbed))
    return np.array(eigs)


def main():
    # Plant: mass-spring-damper type state-space model.
    A = np.array([[0.0, 1.0],
                  [-2.0, -0.4]])
    B = np.array([[0.0],
                  [1.0]])
    x0 = np.array([[1.0],
                   [0.0]])

    pole_sets = {
        "slow": np.array([-1.0 + 1.0j, -1.0 - 1.0j]),
        "medium": np.array([-3.0 + 3.0j, -3.0 - 3.0j]),
        "fast": np.array([-6.0 + 6.0j, -6.0 - 6.0j]),
    }

    print("State-feedback trade-off table")
    print("case       speed       ||K||2       J_u(x0)        poles")
    print("-" * 74)

    trajectories = {}
    for name, poles in pole_sets.items():
        K, Ac, eigvals, speed, k_norm, effort, _ = closed_loop_metrics(A, B, poles, x0)
        print(f"{name:7s} {speed:10.4f} {k_norm:12.4f} {effort:12.4f}   {eigvals}")
        t, x, u = rk4_simulate(Ac, K, x0)
        trajectories[name] = (t, x, u, K, Ac)

    plt.figure()
    for name, (t, x, u, _, _) in trajectories.items():
        plt.plot(t, x[:, 0], label=f"{name}: x1")
    plt.xlabel("time (s)")
    plt.ylabel("state x1")
    plt.title("Faster poles reduce settling time")
    plt.legend()
    plt.grid(True)

    plt.figure()
    for name, (t, x, u, _, _) in trajectories.items():
        plt.plot(t, u, label=f"{name}: u")
    plt.xlabel("time (s)")
    plt.ylabel("control input u")
    plt.title("Faster poles require larger control action")
    plt.legend()
    plt.grid(True)

    for name in ["medium", "fast"]:
        _, _, _, K, _ = trajectories[name]
        eig_cloud = monte_carlo_eigenvalue_sensitivity(A, B, K)
        spread = np.std(eig_cloud.reshape(-1))
        print(f"eigenvalue cloud std for {name:6s}: {spread:.5f}")

    plt.show()


if __name__ == "__main__":
    main()
