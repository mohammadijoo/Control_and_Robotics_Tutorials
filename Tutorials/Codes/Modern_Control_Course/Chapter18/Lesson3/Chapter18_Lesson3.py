# Chapter18_Lesson3.py
# Dynamics associated with Jordan chains: polynomial-exponential terms
#
# Required libraries:
#   pip install numpy matplotlib
# Optional comparison:
#   pip install scipy

import math
import numpy as np
import matplotlib.pyplot as plt


def nilpotent_superdiagonal(n: int) -> np.ndarray:
    """Return N with ones on the first superdiagonal, so N^n = 0."""
    if n <= 0:
        raise ValueError("n must be positive.")
    N = np.zeros((n, n), dtype=float)
    for i in range(n - 1):
        N[i, i + 1] = 1.0
    return N


def jordan_exponential(lam: float, n: int, t: float) -> np.ndarray:
    """
    Compute exp((lam I + N)t) from the finite nilpotent series:
        exp(Jt) = exp(lam t) * sum_{k=0}^{n-1} (t^k / k!) N^k.
    """
    N = nilpotent_superdiagonal(n)
    result = np.eye(n)
    power = np.eye(n)
    for k in range(1, n):
        power = power @ N
        result += (t ** k / math.factorial(k)) * power
    return math.exp(lam * t) * result


def trajectory(lam: float, n: int, z0: np.ndarray, times: np.ndarray) -> np.ndarray:
    """Return rows z(t_i)^T for the Jordan-coordinate system zdot = Jz."""
    z0 = np.asarray(z0, dtype=float).reshape(n)
    return np.vstack([jordan_exponential(lam, n, float(t)) @ z0 for t in times])


def compare_with_scipy(lam: float, n: int, t: float) -> None:
    """Compare the closed-form nilpotent formula with scipy.linalg.expm if SciPy exists."""
    try:
        from scipy.linalg import expm
    except ImportError:
        print("SciPy is not installed; skipping expm comparison.")
        return

    N = nilpotent_superdiagonal(n)
    J = lam * np.eye(n) + N
    err = np.linalg.norm(jordan_exponential(lam, n, t) - expm(J * t), ord=np.inf)
    print(f"Infinity-norm error versus scipy.linalg.expm at t={t}: {err:.3e}")


def main() -> None:
    lam = -0.40
    n = 3
    z0 = np.array([1.0, -0.5, 2.0])
    times = np.linspace(0.0, 12.0, 400)

    Z = trajectory(lam, n, z0, times)

    print("Jordan block size:", n)
    print("lambda:", lam)
    print("Initial Jordan coordinates z0:", z0)
    print("exp(Jt) at t = 2:")
    print(jordan_exponential(lam, n, 2.0))
    compare_with_scipy(lam, n, 2.0)

    plt.figure()
    for i in range(n):
        plt.plot(times, Z[:, i], label=f"z{i + 1}(t)")
    plt.xlabel("time t")
    plt.ylabel("Jordan-coordinate state")
    plt.title("Jordan-chain dynamics: polynomial-exponential terms")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
