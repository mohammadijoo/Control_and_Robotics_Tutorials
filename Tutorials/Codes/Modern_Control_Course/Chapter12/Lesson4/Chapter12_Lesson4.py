# Chapter12_Lesson4.py
# Modern Control — Chapter 12, Lesson 4
# Insights into "Easy" vs "Difficult" States to Reach using controllability Gramians
# Libraries: NumPy, SciPy, Matplotlib

import numpy as np
from scipy.linalg import expm, eigh
import matplotlib.pyplot as plt


def finite_horizon_gramian(A, B, T, steps=4000):
    """Compute Wc(T)=int_0^T exp(A s) B B^T exp(A^T s) ds by trapezoidal quadrature."""
    n = A.shape[0]
    W = np.zeros((n, n), dtype=float)
    ds = T / steps
    for k in range(steps + 1):
        s = k * ds
        Phi = expm(A * s)
        integrand = Phi @ B @ B.T @ Phi.T
        weight = 0.5 if k in (0, steps) else 1.0
        W += weight * integrand * ds
    return 0.5 * (W + W.T)


def minimum_energy(W, delta):
    """Return E_min = delta^T W^{-1} delta."""
    return float(delta.T @ np.linalg.solve(W, delta))


def optimal_input(A, B, W, delta, T, t_grid):
    """u*(t)=B^T exp(A^T(T-t)) W^{-1} delta for steering x(0)=0 to delta."""
    alpha = np.linalg.solve(W, delta)
    values = []
    for t in t_grid:
        values.append((B.T @ expm(A.T * (T - t)) @ alpha).reshape(-1))
    return np.vstack(values)


def plot_energy_ellipse(W, rho=1.0, filename="Chapter12_Lesson4_energy_ellipse.png"):
    """Plot the reachable energy ellipsoid delta^T W^{-1} delta <= rho^2 for n=2."""
    eigvals, eigvecs = eigh(W)
    theta = np.linspace(0, 2 * np.pi, 400)
    circle = np.vstack([np.cos(theta), np.sin(theta)])
    ellipse = eigvecs @ np.diag(rho * np.sqrt(eigvals)) @ circle

    plt.figure(figsize=(6, 5))
    plt.plot(ellipse[0, :], ellipse[1, :], label="energy boundary")
    plt.axhline(0, linewidth=0.8)
    plt.axvline(0, linewidth=0.8)
    for i in range(2):
        axis = eigvecs[:, i] * rho * np.sqrt(eigvals[i])
        plt.plot([0, axis[0]], [0, axis[1]], linewidth=2, label=f"axis {i+1}")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("state component x1")
    plt.ylabel("state component x2")
    plt.title("Finite-horizon controllability energy ellipsoid")
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=160)
    print(f"Saved {filename}")


if __name__ == "__main__":
    # Example: the second state is directly actuated only weakly.
    A = np.array([[-1.0, 0.0],
                  [ 0.0, -4.0]])
    B = np.array([[1.0],
                  [0.08]])
    T = 2.0

    W = finite_horizon_gramian(A, B, T)
    eigvals, eigvecs = eigh(W)
    cond_W = eigvals[-1] / eigvals[0]

    print("Wc(T) =")
    print(W)
    print("Eigenvalues of Wc(T):", eigvals)
    print("Condition number:", cond_W)
    print("Eigenvectors (columns):")
    print(eigvecs)

    # Energy required to reach unit displacement along each Gramian eigenvector.
    for i in range(2):
        q = eigvecs[:, i]
        print(f"Energy to reach q_{i+1}: {minimum_energy(W, q):.6g}")
        print(f"Theoretical value 1/lambda_{i+1}: {1.0 / eigvals[i]:.6g}")

    # Energy for physical coordinate targets.
    e1 = np.array([1.0, 0.0])
    e2 = np.array([0.0, 1.0])
    print("Energy to reach e1:", minimum_energy(W, e1))
    print("Energy to reach e2:", minimum_energy(W, e2))

    # Verify optimal input energy by numerical integration.
    delta = e2
    t_grid = np.linspace(0.0, T, 2001)
    u = optimal_input(A, B, W, delta, T, t_grid)
    numerical_energy = np.trapezoid(np.sum(u * u, axis=1), t_grid)
    print("Numerical energy for target e2:", numerical_energy)
    print("Analytic minimum energy for target e2:", minimum_energy(W, delta))

    plot_energy_ellipse(W, rho=1.0)
