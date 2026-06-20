# Chapter20_Lesson3.py
# System Dynamics (Control Engineering) - Chapter 20, Lesson 3
# Sensitivity to Initial Conditions and Lyapunov Exponents (Intro)

import math
import numpy as np

# -----------------------------
# Part A) Discrete-time map: Logistic map
# x_{n+1} = r x_n (1 - x_n)
# Largest Lyapunov exponent (LLE) approximation:
#   lambda ≈ (1/N) * sum_{n=0}^{N-1} log | f'(x_n) |
# with f'(x) = r (1 - 2x)
# -----------------------------

def logistic_map_step(x: float, r: float) -> float:
    return r * x * (1.0 - x)

def lyapunov_logistic(r: float, x0: float = 0.2, n: int = 200_000, discard: int = 5_000) -> float:
    x = float(x0)
    # transient removal
    for _ in range(discard):
        x = logistic_map_step(x, r)

    s = 0.0
    for _ in range(n):
        # derivative along the orbit
        fp = r * (1.0 - 2.0 * x)
        s += math.log(abs(fp) + 1e-300)  # tiny offset avoids log(0) in degenerate cases
        x = logistic_map_step(x, r)
    return s / n

# -----------------------------
# Part B) Continuous-time system: Lorenz (classic chaotic ODE)
#   xdot = sigma (y - x)
#   ydot = x (rho - z) - y
#   zdot = x y - beta z
#
# Largest Lyapunov exponent approximation by "two-trajectory renormalization":
# 1) Integrate a reference state X(t)
# 2) Integrate a perturbed state Xp(t) = X(t) + d0 * u (small separation)
# 3) Every m steps: measure separation d = ||Xp - X||,
#    accumulate log(d/d0), then renormalize: Xp = X + d0 * (Xp - X)/d
# 4) lambda ≈ (1/T) * sum log(d/d0)
# -----------------------------

def lorenz_rhs(t: float, X: np.ndarray, sigma: float = 10.0, rho: float = 28.0, beta: float = 8.0/3.0) -> np.ndarray:
    x, y, z = X
    return np.array([
        sigma * (y - x),
        x * (rho - z) - y,
        x * y - beta * z
    ], dtype=float)

def rk4_step(rhs, t: float, y: np.ndarray, dt: float, *rhs_args, **rhs_kwargs) -> np.ndarray:
    k1 = rhs(t, y, *rhs_args, **rhs_kwargs)
    k2 = rhs(t + 0.5*dt, y + 0.5*dt*k1, *rhs_args, **rhs_kwargs)
    k3 = rhs(t + 0.5*dt, y + 0.5*dt*k2, *rhs_args, **rhs_kwargs)
    k4 = rhs(t + dt, y + dt*k3, *rhs_args, **rhs_kwargs)
    return y + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

def lyapunov_lorenz_lle(
    X0 = (1.0, 1.0, 1.0),
    dt: float = 0.01,
    T: float = 100.0,
    transient: float = 10.0,
    renorm_every: int = 10,
    d0: float = 1e-8,
    sigma: float = 10.0,
    rho: float = 28.0,
    beta: float = 8.0/3.0,
    seed: int = 0
) -> float:
    rng = np.random.default_rng(seed)
    X = np.array(X0, dtype=float)

    # random unit direction for initial perturbation
    u = rng.normal(size=3)
    u = u / np.linalg.norm(u)
    Xp = X + d0 * u

    # integrate out transient without measuring
    t = 0.0
    n_trans = int(transient / dt)
    for _ in range(n_trans):
        X = rk4_step(lorenz_rhs, t, X, dt, sigma=sigma, rho=rho, beta=beta)
        Xp = rk4_step(lorenz_rhs, t, Xp, dt, sigma=sigma, rho=rho, beta=beta)
        t += dt
        # keep perturbation small to avoid collapse or blow-up
        d = np.linalg.norm(Xp - X)
        if d == 0.0:
            Xp = X + d0 * u
        else:
            Xp = X + d0 * (Xp - X) / d

    # main accumulation
    s = 0.0
    steps = int(T / dt)
    count_renorm = 0
    for k in range(steps):
        X = rk4_step(lorenz_rhs, t, X, dt, sigma=sigma, rho=rho, beta=beta)
        Xp = rk4_step(lorenz_rhs, t, Xp, dt, sigma=sigma, rho=rho, beta=beta)
        t += dt

        if (k + 1) % renorm_every == 0:
            d = np.linalg.norm(Xp - X)
            if d == 0.0:
                # extremely rare; re-seed perturbation
                u = rng.normal(size=3)
                u = u / np.linalg.norm(u)
                Xp = X + d0 * u
                continue
            s += math.log(d / d0)
            count_renorm += 1
            Xp = X + d0 * (Xp - X) / d

    total_time = count_renorm * renorm_every * dt
    return s / total_time

def demo():
    print("Logistic map LLE examples:")
    for r in [3.2, 3.5, 3.9, 4.0]:
        lam = lyapunov_logistic(r, x0=0.234, n=100_000, discard=5_000)
        print(f"  r={r:>3}: lambda ≈ {lam:.6f}")

    print("\nLorenz LLE example (classic sigma=10, rho=28, beta=8/3):")
    lam_lor = lyapunov_lorenz_lle(T=120.0, transient=20.0, dt=0.01, renorm_every=10, d0=1e-8)
    print(f"  lambda_max ≈ {lam_lor:.4f} 1/time-unit")

if __name__ == "__main__":
    demo()
