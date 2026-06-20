"""
Chapter4_Lesson2.py
Autonomous Mobile Robots — Chapter 4 Lesson 2
Slip, Skid, and Terrain Interaction Models

This script provides:
1) Hard-ground wheel slip model with combined-slip saturation (friction ellipse).
2) Simple soft-soil (terramechanics-inspired) traction surrogate.
3) A small time-domain simulation of a driven wheel + 1D vehicle mass.

Dependencies: numpy, matplotlib (standard scientific Python stack).
"""

from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt


def slip_ratio(vx: float, omega: float, R: float, eps: float = 1e-6) -> float:
    """
    Longitudinal slip ratio kappa in a symmetric definition:

        kappa = (R*omega - vx) / max(|vx|, |R*omega|, eps)

    Positive kappa corresponds to driving (wheel tends to spin faster than ground speed).
    Negative kappa corresponds to braking (wheel tends to rotate slower than ground speed).
    """
    denom = max(abs(vx), abs(R * omega), eps)
    return (R * omega - vx) / denom


def slip_angle(vx: float, vy: float, eps: float = 1e-6) -> float:
    """
    Lateral slip angle alpha = atan2(vy, |vx| + eps).
    """
    return np.arctan2(vy, abs(vx) + eps)


def friction_ellipse_saturate(Fx_lin: float, Fy_lin: float, mu: float, Fz: float) -> tuple[float, float]:
    """
    Enforce (Fx/(mu Fz))^2 + (Fy/(mu Fz))^2 <= 1 by radial scaling.

    This is the Euclidean projection onto a scaled L2 ball in normalized coordinates,
    which equals radial scaling for this particular constraint set.
    """
    cap = max(mu * Fz, 0.0)
    if cap <= 0.0:
        return 0.0, 0.0

    nx = Fx_lin / cap
    ny = Fy_lin / cap
    r2 = nx * nx + ny * ny
    if r2 <= 1.0:
        return Fx_lin, Fy_lin

    s = 1.0 / np.sqrt(r2)
    return s * Fx_lin, s * Fy_lin


def hard_ground_forces(vx: float, vy: float, omega: float, R: float,
                       Fz: float, mu: float,
                       Ck: float = 15000.0, Ca: float = 12000.0) -> tuple[float, float, float, float]:
    """
    Simple hard-ground tire model:
      Fx = Ck * kappa,  Fy = -Ca * alpha  (small-slip linearization)
      then saturate with friction ellipse.

    Returns: Fx, Fy, kappa, alpha
    """
    kappa = slip_ratio(vx, omega, R)
    alpha = slip_angle(vx, vy)
    Fx_lin = Ck * kappa
    Fy_lin = -Ca * alpha
    Fx, Fy = friction_ellipse_saturate(Fx_lin, Fy_lin, mu, Fz)
    return Fx, Fy, kappa, alpha


def soft_soil_traction_surrogate(kappa: float, Fz: float, mu_peak: float = 0.65, k_shape: float = 8.0) -> float:
    """
    A compact terramechanics-inspired surrogate for longitudinal traction:
        Fx = Fz * mu_eff(kappa),
        mu_eff(kappa) = mu_peak * (1 - exp(-k_shape * |kappa|)) * sign(kappa)

    This captures:
      - approximately linear traction near kappa = 0,
      - saturation to a peak traction coefficient as |kappa| increases.

    It is NOT a full Bekker + Janosi-Hanamoto integral model, but is useful for control-level simulation.
    """
    s = np.sign(kappa) if kappa != 0.0 else 0.0
    mu_eff = mu_peak * (1.0 - np.exp(-k_shape * abs(kappa))) * s
    return Fz * mu_eff


def simulate_1d_wheel_vehicle(
    T_cmd: float = 12.0,
    terrain: str = "hard",
    t_end: float = 4.0,
    dt: float = 1e-3,
) -> dict[str, np.ndarray]:
    """
    1D model: vehicle longitudinal speed vx, wheel spin omega.

    Dynamics:
      m * dvx/dt = Fx - Frr
      Iw * domega/dt = T_cmd - R*Fx - bw*omega

    For 'hard' terrain: Fx from linear tire + friction ellipse.
    For 'soft' terrain: Fx from traction surrogate.

    Returns time histories in a dictionary.
    """
    # Parameters (typical small AMR)
    m = 25.0           # kg
    R = 0.10           # m
    Iw = 0.05          # kg m^2 (wheel+gear equivalent)
    bw = 0.02          # N m s/rad viscous loss
    g = 9.81

    # Normal load per wheel (assume one driven wheel supporting part of mass)
    Fz = 0.25 * m * g

    # Rolling resistance
    Crr = 0.02
    Frr = Crr * Fz

    # Hard-ground friction
    mu = 0.8

    N = int(t_end / dt) + 1
    t = np.linspace(0.0, t_end, N)
    vx = np.zeros(N)
    omega = np.zeros(N)
    kappa = np.zeros(N)
    Fx = np.zeros(N)

    # Initial conditions
    vx[0] = 0.0
    omega[0] = 0.0

    for i in range(N - 1):
        if terrain == "hard":
            Fx_i, _, k_i, _ = hard_ground_forces(vx[i], 0.0, omega[i], R, Fz, mu)
        elif terrain == "soft":
            k_i = slip_ratio(vx[i], omega[i], R)
            Fx_i = soft_soil_traction_surrogate(k_i, Fz, mu_peak=0.55, k_shape=10.0)
            # Soil also increases motion resistance: raise effective rolling resistance a bit
            Fx_i -= 0.015 * Fz * np.sign(vx[i])
        else:
            raise ValueError("terrain must be 'hard' or 'soft'.")

        # Vehicle
        ax = (Fx_i - Frr) / m
        vx[i + 1] = vx[i] + dt * ax

        # Wheel
        domega = (T_cmd - R * Fx_i - bw * omega[i]) / Iw
        omega[i + 1] = omega[i] + dt * domega

        Fx[i] = Fx_i
        kappa[i] = k_i

    Fx[-1] = Fx[-2]
    kappa[-1] = kappa[-2]

    return dict(t=t, vx=vx, omega=omega, kappa=kappa, Fx=Fx, R=np.array([R]), Fz=np.array([Fz]))


def main() -> None:
    hard = simulate_1d_wheel_vehicle(T_cmd=12.0, terrain="hard")
    soft = simulate_1d_wheel_vehicle(T_cmd=12.0, terrain="soft")

    # Plot
    plt.figure()
    plt.plot(hard["t"], hard["vx"], label="vx hard")
    plt.plot(soft["t"], soft["vx"], label="vx soft")
    plt.xlabel("time [s]")
    plt.ylabel("vx [m/s]")
    plt.grid(True)
    plt.legend()

    plt.figure()
    plt.plot(hard["t"], hard["kappa"], label="kappa hard")
    plt.plot(soft["t"], soft["kappa"], label="kappa soft")
    plt.xlabel("time [s]")
    plt.ylabel("slip ratio kappa [-]")
    plt.grid(True)
    plt.legend()

    plt.figure()
    plt.plot(hard["t"], hard["Fx"], label="Fx hard")
    plt.plot(soft["t"], soft["Fx"], label="Fx soft")
    plt.xlabel("time [s]")
    plt.ylabel("Fx [N]")
    plt.grid(True)
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()
