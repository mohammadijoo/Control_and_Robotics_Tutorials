# Chapter19_Lesson2.py
# Separation of Variables & Modal Decomposition (1D Heat Equation, Dirichlet BC)
#
# PDE: u_t = alpha * u_xx + b(x) * u_in(t),   x in (0, L)
# BC:  u(0,t)=0, u(L,t)=0
# IC:  u(x,0)=u0(x)
#
# Modal expansion: u(x,t) = sum_{n=1..N} q_n(t) * sin(n*pi*x/L)
# ODEs: q_n'(t) = -alpha*(n*pi/L)^2 * q_n(t) + B_n * u_in(t)
#
# Requires: numpy, scipy, matplotlib

import numpy as np
from math import pi
from dataclasses import dataclass

try:
    from scipy.integrate import quad
    from scipy.integrate import solve_ivp
except Exception as e:
    raise RuntimeError("This script requires SciPy (scipy.integrate). Install with: pip install scipy") from e

@dataclass
class Heat1DModal:
    L: float = 1.0
    alpha: float = 0.02
    N: int = 30  # number of modes

    def phi(self, n, x):
        return np.sin(n * pi * x / self.L)

    def lam(self, n):
        return (n * pi / self.L) ** 2

    def project(self, func, n):
        # a_n = (2/L) * ∫_0^L func(x) * sin(n*pi*x/L) dx
        L = self.L
        integrand = lambda x: func(x) * np.sin(n * pi * x / L)
        val, _ = quad(integrand, 0.0, L, limit=200)
        return (2.0 / L) * val

    def build_matrices(self, b_func):
        # diagonal Lambda, and input projection vector B
        n = np.arange(1, self.N + 1)
        Lambda = np.diag((n * pi / self.L) ** 2)
        B = np.array([self.project(b_func, int(k)) for k in n])
        return Lambda, B

    def simulate(self, u0_func, b_func, u_in, t_span=(0.0, 5.0), t_eval=None):
        # initial modal coordinates q(0)
        q0 = np.array([self.project(u0_func, n) for n in range(1, self.N + 1)])

        Lambda, B = self.build_matrices(b_func)

        def ode(t, q):
            return -self.alpha * (Lambda @ q) + B * u_in(t)

        if t_eval is None:
            t_eval = np.linspace(t_span[0], t_span[1], 300)

        sol = solve_ivp(ode, t_span, q0, t_eval=t_eval, method="RK45")
        if not sol.success:
            raise RuntimeError("ODE solve failed: " + str(sol.message))
        return sol.t, sol.y, q0, Lambda, B

    def reconstruct(self, x_grid, q):
        # u(x) from modal coefficients q at a given time
        x = np.asarray(x_grid)
        u = np.zeros_like(x, dtype=float)
        for n in range(1, self.N + 1):
            u += q[n - 1] * np.sin(n * pi * x / self.L)
        return u


def main():
    model = Heat1DModal(L=1.0, alpha=0.02, N=40)

    # Initial condition and distributed input shape
    u0 = lambda x: x * (1.0 - x)          # smooth, satisfies Dirichlet at x=0,1
    b  = lambda x: 4.0 * x * (1.0 - x)    # actuation profile (also Dirichlet-compatible)

    # Input signal
    omega = 2.0
    u_in = lambda t: 0.8 * np.sin(omega * t)

    t, q_hist, q0, Lambda, B = model.simulate(u0, b, u_in, t_span=(0.0, 6.0))

    # Reconstruct solution at a few time instants
    x = np.linspace(0.0, model.L, 200)
    sample_times = [0.0, 1.0, 2.5, 5.0, 6.0]
    idx = [int(np.argmin(np.abs(t - ts))) for ts in sample_times]

    # Plot
    import matplotlib.pyplot as plt

    for k, ts in zip(idx, sample_times):
        u_xt = model.reconstruct(x, q_hist[:, k])
        plt.plot(x, u_xt, label=f"t={t[k]:.2f}")

    plt.xlabel("x")
    plt.ylabel("u(x,t)")
    plt.title("1D Heat equation via modal decomposition (Dirichlet BC)")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Print first few modal time constants
    n = np.arange(1, 6)
    tau = 1.0 / (model.alpha * (n * pi / model.L) ** 2)
    print("First 5 modal decay time constants:", tau)

    # Example: compare truncation levels at final time
    t_final_idx = -1
    u_true = model.reconstruct(x, q_hist[:, t_final_idx])

    for Ntest in [3, 6, 12, 24, model.N]:
        modelN = Heat1DModal(L=model.L, alpha=model.alpha, N=Ntest)
        # reuse the first Ntest coefficients from the simulated solution
        qN = q_hist[:Ntest, t_final_idx]
        uN = modelN.reconstruct(x, qN)
        err = np.linalg.norm(u_true - uN) / np.linalg.norm(u_true)
        print(f"Relative L2 error with N={Ntest:2d}: {err:.3e}")

if __name__ == "__main__":
    main()
