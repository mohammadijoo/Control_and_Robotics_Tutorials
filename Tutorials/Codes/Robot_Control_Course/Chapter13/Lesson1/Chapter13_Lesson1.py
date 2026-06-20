
import numpy as np
import matplotlib.pyplot as plt

def nominal_control(x, x_ref, k_p=2.0):
    # Simple proportional controller (no safety)
    return -k_p * (x - x_ref)

def cbf_filter_1d(x, u_nom, gamma=5.0):
    # CBF constraint: u >= -gamma * x
    u_min = -gamma * x
    return max(u_nom, u_min)

def simulate_cbf_1d(x0, x_ref, dt=0.001, T=2.0, k_p=2.0, gamma=5.0):
    N = int(T / dt)
    xs = np.zeros(N + 1)
    us_nom = np.zeros(N)
    us_safe = np.zeros(N)
    t = np.linspace(0.0, T, N + 1)
    xs[0] = x0

    for k in range(N):
        x = xs[k]
        u_nom = nominal_control(x, x_ref, k_p)
        u_safe = cbf_filter_1d(x, u_nom, gamma)

        us_nom[k] = u_nom
        us_safe[k] = u_safe

        # Integrate dynamics: x_dot = u_safe
        xs[k + 1] = x + dt * u_safe

    return t, xs, us_nom, us_safe

if __name__ == "__main__":
    # Start inside the safe set but close to the boundary
    x0 = 0.05
    x_ref = -1.0  # nominal controller wants to cross into unsafe region

    t, xs, u_nom, u_safe = simulate_cbf_1d(x0, x_ref)

    plt.figure()
    plt.plot(t, xs)
    plt.axhline(0.0, linestyle="--")  # safety boundary
    plt.xlabel("time [s]")
    plt.ylabel("x(t)")
    plt.title("State trajectory with CBF safety filter")

    plt.figure()
    plt.plot(t[:-1], u_nom, label="u_nom")
    plt.plot(t[:-1], u_safe, label="u_safe")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("time [s]")
    plt.ylabel("control input")
    plt.legend()
    plt.show()
