"""
Chapter29_Lesson3.py

Numerical experiments for Lesson 3:
Stability notions for continuous-time linear time-varying (LTV) systems.

The examples compare:
1. Uniform exponential stability for a scalar LTV system.
2. Uniform stability without uniform attractivity.
3. Numerical estimation of a state-transition matrix for a 2x2 LTV system.

Dependencies:
    numpy
"""

import math
import numpy as np


def scalar_phi_by_quadrature(a_func, t0, t1, steps=2000):
    """Approximate Phi(t1,t0) for x_dot = a(t) x."""
    if t1 < t0:
        raise ValueError("This routine assumes t1 >= t0.")
    if t1 == t0:
        return 1.0

    grid = np.linspace(t0, t1, steps + 1)
    vals = np.array([a_func(t) for t in grid])
    integral = np.trapz(vals, grid)
    return math.exp(integral)


def phi_uniform_exponential_scalar(t, t0):
    """
    Exact transition for x_dot = (-0.5 + 0.25 sin(t)) x.

    Integral from t0 to t:
        -0.5 (t-t0) + 0.25 (cos(t0)-cos(t)).
    Since cos(t0)-cos(t) <= 2, |Phi| <= exp(0.5) exp(-0.5(t-t0)).
    """
    return math.exp(-0.5 * (t - t0) + 0.25 * (math.cos(t0) - math.cos(t)))


def phi_uniform_stable_not_uniform_attractive(t, t0):
    """
    Exact transition for x_dot = -1/(1+t) x:
        Phi(t,t0) = (1+t0)/(1+t).

    It is uniformly stable because |Phi| <= 1 for t >= t0.
    It is not uniformly attractive because for fixed T,
        sup_t0 Phi(t0+T,t0) = sup_t0 (1+t0)/(1+t0+T) = 1.
    """
    return (1.0 + t0) / (1.0 + t)


def matmul(A, B):
    return A @ B


def ltv_matrix_A(t):
    """
    A bounded 2x2 time-varying matrix.
    The damping term is positive and periodic, so the system is a useful
    numerical example for examining transition-matrix norm decay.
    """
    k = 1.0 + 0.20 * math.sin(t)
    c = 0.80 + 0.10 * math.cos(2.0 * t)
    return np.array([[0.0, 1.0], [-k, -c]], dtype=float)


def rk4_phi(A_func, t0, t1, h=0.002):
    """
    Integrate Phi_dot = A(t) Phi with Phi(t0,t0)=I using RK4.
    """
    if t1 < t0:
        raise ValueError("This routine assumes t1 >= t0.")
    n = int(math.ceil((t1 - t0) / h))
    if n == 0:
        return np.eye(2)

    h_eff = (t1 - t0) / n
    t = t0
    Phi = np.eye(2)

    def f(time, X):
        return A_func(time) @ X

    for _ in range(n):
        K1 = f(t, Phi)
        K2 = f(t + 0.5 * h_eff, Phi + 0.5 * h_eff * K1)
        K3 = f(t + 0.5 * h_eff, Phi + 0.5 * h_eff * K2)
        K4 = f(t + h_eff, Phi + h_eff * K3)
        Phi = Phi + (h_eff / 6.0) * (K1 + 2.0 * K2 + 2.0 * K3 + K4)
        t += h_eff

    return Phi


def estimate_transition_norms():
    """
    Print sample transition-matrix norms over several initial times and horizons.
    """
    print("Scalar uniformly exponentially stable example")
    M = math.exp(0.5)
    alpha = 0.5
    for tau in [0, 1, 2, 5, 10]:
        ratios = []
        for t0 in np.linspace(0, 20, 41):
            t = t0 + tau
            phi = abs(phi_uniform_exponential_scalar(t, t0))
            bound = M * math.exp(-alpha * tau)
            ratios.append(phi / bound)
        print(f"tau={tau:>2}, max |Phi|/bound = {max(ratios):.6f}")

    print("\nUniformly stable but not uniformly attractive example")
    for T in [1, 5, 20]:
        values = [phi_uniform_stable_not_uniform_attractive(t0 + T, t0)
                  for t0 in [0, 10, 100, 1000, 10000]]
        print(f"T={T:>2}, Phi(t0+T,t0) for growing t0 = {values}")

    print("\n2x2 transition-matrix norm estimates")
    for tau in [1, 2, 5, 10]:
        norms = []
        for t0 in np.linspace(0, 5, 6):
            Phi = rk4_phi(ltv_matrix_A, t0, t0 + tau, h=0.005)
            norms.append(np.linalg.norm(Phi, 2))
        print(f"tau={tau:>2}, max sampled ||Phi||_2 = {max(norms):.6f}")


if __name__ == "__main__":
    estimate_transition_norms()
