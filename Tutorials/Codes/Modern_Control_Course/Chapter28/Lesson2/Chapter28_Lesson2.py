# Chapter28_Lesson2.py
"""
Energy-like signal measures for a simple continuous-time state-space model.

Libraries used in modern control workflows:
- numpy: arrays and numerical linear algebra
- scipy.integrate: ODE simulation
- scipy.linalg: Lyapunov equations, matrix functions, eigen-analysis
- python-control (optional): state-space objects and system norms

This script intentionally computes most quantities from sampled data so that the
connection between signal norms and numerical simulation is visible.
"""

from __future__ import annotations

import numpy as np
from scipy.integrate import solve_ivp


def trapz_energy(t: np.ndarray, y: np.ndarray, W: np.ndarray | None = None) -> float:
    """Approximate integral y(t)^T W y(t) dt using the trapezoidal rule."""
    if W is None:
        W = np.eye(y.shape[1])
    values = np.einsum("ij,jk,ik->i", y, W, y)
    return float(np.trapz(values, t))


def l2_norm(t: np.ndarray, y: np.ndarray, W: np.ndarray | None = None) -> float:
    """Weighted L2 norm: sqrt(integral y^T W y dt)."""
    return float(np.sqrt(max(trapz_energy(t, y, W), 0.0)))


def rms_value(t: np.ndarray, y: np.ndarray, W: np.ndarray | None = None) -> float:
    """Weighted RMS value over a finite horizon [t0, tf]."""
    horizon = float(t[-1] - t[0])
    if horizon <= 0.0:
        raise ValueError("The time vector must span a positive interval.")
    return float(np.sqrt(max(trapz_energy(t, y, W) / horizon, 0.0)))


def linf_norm(y: np.ndarray) -> float:
    """Vector-valued L-infinity norm: sup_t ||y(t)||_2."""
    return float(np.max(np.linalg.norm(y, axis=1)))


def simulate_mass_spring_damper() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Simulate x_dot = A x + B u, y = C x with a decaying sinusoidal input."""
    A = np.array([[0.0, 1.0], [-4.0, -0.8]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[1.0, 0.0], [0.0, 1.0]])
    x0 = np.array([1.0, 0.0])

    def u(t: float) -> float:
        return 0.5 * np.sin(3.0 * t) * np.exp(-0.2 * t)

    def rhs(t: float, x: np.ndarray) -> np.ndarray:
        return A @ x + B[:, 0] * u(t)

    t_eval = np.linspace(0.0, 12.0, 1201)
    sol = solve_ivp(rhs, (t_eval[0], t_eval[-1]), x0, t_eval=t_eval, rtol=1e-9, atol=1e-11)
    if not sol.success:
        raise RuntimeError(sol.message)

    x = sol.y.T
    y = x @ C.T
    u_samples = np.array([[u(t)] for t in t_eval])
    return t_eval, x, y, u_samples


def main() -> None:
    t, x, y, u = simulate_mass_spring_damper()

    Qx = np.diag([10.0, 1.0])       # position is penalized more than velocity
    Ru = np.array([[0.2]])          # modest input penalty
    Wy = np.diag([1.0, 0.1])        # output energy metric

    state_energy = trapz_energy(t, x, Qx)
    input_energy = trapz_energy(t, u, Ru)
    performance_index = state_energy + input_energy

    print("Weighted state energy       =", state_energy)
    print("Weighted input energy       =", input_energy)
    print("Quadratic performance J     =", performance_index)
    print("Weighted output L2 norm     =", l2_norm(t, y, Wy))
    print("Weighted output RMS         =", rms_value(t, y, Wy))
    print("Output L-infinity norm      =", linf_norm(y))
    print("Unweighted input L2 norm    =", l2_norm(t, u))

    # Optional: show how a sampled induced-gain estimate is formed for this one input.
    gain_estimate = l2_norm(t, y) / max(l2_norm(t, u), 1e-12)
    print("Sampled input-output L2 gain estimate for this input =", gain_estimate)


if __name__ == "__main__":
    main()
