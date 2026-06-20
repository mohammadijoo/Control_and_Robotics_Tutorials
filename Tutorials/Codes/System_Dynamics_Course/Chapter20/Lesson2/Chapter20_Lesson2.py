# Chapter20_Lesson2.py
# System Dynamics — Chapter 20 (Chaos, Complex Dynamics, and Computational Tools)
# Lesson 2: Bifurcation Diagrams, Period Doubling, and Strange Attractors
#
# Outputs:
#   - logistic_bifurcation.png (bifurcation diagram)
#   - logistic_bifurcation.csv (points used in the diagram)
#   - lorenz_attractor.png (3D trajectory projection)
#   - lorenz_traj.csv (Lorenz trajectory samples)

import numpy as np
import matplotlib.pyplot as plt

def logistic_map(x, r):
    return r * x * (1.0 - x)

def bifurcation_logistic(r_min=2.5, r_max=4.0, n_r=6000,
                         n_transient=1200, n_keep=200, x0=0.123456):
    rs = np.linspace(r_min, r_max, n_r)
    x = np.full(n_r, x0, dtype=float)

    # Burn-in / transient
    for _ in range(n_transient):
        x = logistic_map(x, rs)

    # Keep last n_keep iterates for each r
    r_out = np.repeat(rs, n_keep)
    x_out = np.empty(n_r * n_keep, dtype=float)
    idx = 0
    for _ in range(n_keep):
        x = logistic_map(x, rs)
        x_out[idx:idx+n_r] = x
        idx += n_r

    # Save CSV (r, x)
    data = np.column_stack([r_out, x_out])
    np.savetxt("logistic_bifurcation.csv", data, delimiter=",", header="r,x", comments="")

    # Plot
    plt.figure(figsize=(10, 5))
    plt.plot(r_out, x_out, ",", markersize=1)
    plt.xlabel("r")
    plt.ylabel("x (asymptotic samples)")
    plt.title("Logistic map bifurcation diagram")
    plt.tight_layout()
    plt.savefig("logistic_bifurcation.png", dpi=200)
    plt.close()

def rk4_step(f, t, y, h):
    k1 = f(t, y)
    k2 = f(t + 0.5*h, y + 0.5*h*k1)
    k3 = f(t + 0.5*h, y + 0.5*h*k2)
    k4 = f(t + h, y + h*k3)
    return y + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4)

def lorenz_rhs(sigma=10.0, rho=28.0, beta=8.0/3.0):
    def f(t, y):
        x, yv, z = y
        return np.array([
            sigma*(yv - x),
            x*(rho - z) - yv,
            x*yv - beta*z
        ], dtype=float)
    return f

def simulate_lorenz(T=40.0, h=0.005, y0=(1.0, 1.0, 1.0),
                    sigma=10.0, rho=28.0, beta=8.0/3.0,
                    transient=5.0, sample_stride=4):
    f = lorenz_rhs(sigma, rho, beta)
    n = int(T / h)
    y = np.array(y0, dtype=float)
    t = 0.0

    # Run and store after transient
    traj = []
    for i in range(n):
        y = rk4_step(f, t, y, h)
        t += h
        if t >= transient and (i % sample_stride == 0):
            traj.append((t, y[0], y[1], y[2]))
    traj = np.array(traj, dtype=float)
    np.savetxt("lorenz_traj.csv", traj, delimiter=",", header="t,x,y,z", comments="")

    # Plot 3D trajectory (requires mpl_toolkits)
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(traj[:,1], traj[:,2], traj[:,3], linewidth=0.4)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("Lorenz attractor trajectory (sampled)")
    plt.tight_layout()
    plt.savefig("lorenz_attractor.png", dpi=200)
    plt.close()

if __name__ == "__main__":
    bifurcation_logistic()
    simulate_lorenz()
    print("Done. Generated PNGs and CSVs in the current directory.")
