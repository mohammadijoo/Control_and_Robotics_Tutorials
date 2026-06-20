# Chapter20_Lesson1.py
"""
System Dynamics — Chapter 20, Lesson 1
Nonlinear Maps and Continuous-Time Chaotic Systems
(Logistic Map, Lorenz System)

This script:
1) Simulates the logistic map (time series + cobweb plot).
2) Simulates the Lorenz system using (a) from-scratch RK4 and (b) SciPy solve_ivp (optional).
3) Demonstrates sensitivity to initial conditions by comparing nearby initial states.

Dependencies:
- numpy
- matplotlib
Optional:
- scipy (for solve_ivp)
"""

import numpy as np
import matplotlib.pyplot as plt


# -----------------------------
# Logistic map: x_{n+1} = r x_n (1 - x_n)
# -----------------------------
def logistic_map(r: float, x0: float, n: int) -> np.ndarray:
    x = np.empty(n + 1, dtype=float)
    x[0] = x0
    for k in range(n):
        x[k + 1] = r * x[k] * (1.0 - x[k])
    return x


def cobweb_data(f, x0: float, n: int):
    """Return arrays for cobweb plotting: (x, y) polyline points."""
    xs = []
    ys = []
    x = x0
    for _ in range(n):
        y = f(x)
        # vertical: (x, x) -> (x, y)
        xs.extend([x, x])
        ys.extend([x, y])
        # horizontal: (x, y) -> (y, y)
        xs.extend([x, y])
        ys.extend([y, y])
        x = y
    return np.array(xs), np.array(ys)


def plot_logistic_demo(r=3.8, x0=0.2, n=80):
    f = lambda x: r * x * (1.0 - x)
    x = logistic_map(r, x0, n)

    # Time series
    plt.figure()
    plt.plot(np.arange(n + 1), x)
    plt.xlabel("n")
    plt.ylabel("x_n")
    plt.title(f"Logistic map time series (r={r}, x0={x0})")
    plt.grid(True)

    # Cobweb
    plt.figure()
    grid = np.linspace(0, 1, 600)
    plt.plot(grid, f(grid), label="f(x)")
    plt.plot(grid, grid, label="y=x")
    xs, ys = cobweb_data(f, x0, n=40)
    plt.plot(xs, ys, linewidth=1.0)
    plt.scatter([x0], [0], s=20)
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.xlabel("x_n")
    plt.ylabel("x_{n+1}")
    plt.title(f"Cobweb plot (r={r}, x0={x0})")
    plt.legend()
    plt.grid(True)


# -----------------------------
# Lorenz system
#   x' = sigma (y - x)
#   y' = x (rho - z) - y
#   z' = x y - beta z
# -----------------------------
def lorenz_rhs(t, state, sigma=10.0, rho=28.0, beta=8.0/3.0):
    x, y, z = state
    dx = sigma * (y - x)
    dy = x * (rho - z) - y
    dz = x * y - beta * z
    return np.array([dx, dy, dz], dtype=float)


def rk4_step(f, t, x, h, **params):
    k1 = f(t, x, **params)
    k2 = f(t + 0.5*h, x + 0.5*h*k1, **params)
    k3 = f(t + 0.5*h, x + 0.5*h*k2, **params)
    k4 = f(t + h, x + h*k3, **params)
    return x + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4)


def integrate_lorenz_rk4(
    x0=(1.0, 1.0, 1.0),
    t0=0.0,
    tf=40.0,
    h=0.005,
    sigma=10.0,
    rho=28.0,
    beta=8.0/3.0
):
    n = int(np.ceil((tf - t0) / h))
    t = np.linspace(t0, t0 + n*h, n + 1)
    X = np.empty((n + 1, 3), dtype=float)
    X[0, :] = np.array(x0, dtype=float)
    for k in range(n):
        X[k + 1, :] = rk4_step(lorenz_rhs, t[k], X[k, :], h,
                               sigma=sigma, rho=rho, beta=beta)
    return t, X


def plot_lorenz_demo():
    t, X = integrate_lorenz_rk4()
    x, y, z = X[:, 0], X[:, 1], X[:, 2]

    plt.figure()
    plt.plot(t, x, label="x(t)")
    plt.plot(t, y, label="y(t)")
    plt.plot(t, z, label="z(t)")
    plt.xlabel("t")
    plt.ylabel("states")
    plt.title("Lorenz system states vs time (RK4)")
    plt.legend()
    plt.grid(True)

    # 3D attractor
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(x, y, z, linewidth=0.7)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("Lorenz attractor (RK4)")


def sensitivity_demo_lorenz(eps=1e-9):
    # Two nearby initial conditions
    x0 = np.array([1.0, 1.0, 1.0])
    x0b = x0 + np.array([eps, 0.0, 0.0])

    t, Xa = integrate_lorenz_rk4(x0=tuple(x0))
    _, Xb = integrate_lorenz_rk4(x0=tuple(x0b))

    d = np.linalg.norm(Xa - Xb, axis=1)

    plt.figure()
    plt.semilogy(t, d + 1e-30)
    plt.xlabel("t")
    plt.ylabel("||delta(t)||")
    plt.title(f"Sensitivity in Lorenz system (eps={eps})")
    plt.grid(True)


def optional_scipy_solve_ivp_demo():
    try:
        from scipy.integrate import solve_ivp
    except Exception:
        print("SciPy not available; skipping solve_ivp demo.")
        return

    def rhs(t, y):  # wrapper for solve_ivp signature
        return lorenz_rhs(t, y, sigma=10.0, rho=28.0, beta=8.0/3.0)

    sol = solve_ivp(rhs, t_span=(0.0, 40.0), y0=[1.0, 1.0, 1.0],
                    max_step=0.01, rtol=1e-9, atol=1e-12)

    plt.figure()
    plt.plot(sol.t, sol.y[0], label="x(t)")
    plt.plot(sol.t, sol.y[1], label="y(t)")
    plt.plot(sol.t, sol.y[2], label="z(t)")
    plt.xlabel("t")
    plt.ylabel("states")
    plt.title("Lorenz system states vs time (solve_ivp)")
    plt.legend()
    plt.grid(True)


if __name__ == "__main__":
    plot_logistic_demo()
    plot_lorenz_demo()
    sensitivity_demo_lorenz()
    optional_scipy_solve_ivp_demo()
    plt.show()
