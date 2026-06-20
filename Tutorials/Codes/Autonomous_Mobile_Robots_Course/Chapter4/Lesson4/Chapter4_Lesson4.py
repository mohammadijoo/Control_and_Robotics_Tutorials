# Chapter4_Lesson4.py
"""
Autonomous Mobile Robots (Control Engineering)
Chapter 4: Mobile Robot Dynamics (Applied)
Lesson 4: Stability and Tip-Over Risk (vehicle view)

This script implements quasi-static tip-over / sliding checks for a ground robot
based on vehicle geometry (track width, wheelbase, CoG height) and friction.

Key outputs:
- Load Transfer Ratio (LTR) for lateral and longitudinal acceleration
- Stability margins and conservative safe speed bounds along a curved path

Dependencies: numpy, matplotlib (optional for plotting)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass(frozen=True)
class VehicleParams:
    track: float          # [m] distance between left and right wheel contact lines
    wheelbase: float      # [m] distance between front and rear axle contact lines
    cg_height: float      # [m] CoG height above ground plane
    mu: float = 0.8       # [-] friction coefficient (lateral/longitudinal, simplified)
    g: float = 9.81       # [m/s^2] gravity magnitude


def lateral_accel(v: np.ndarray, kappa: np.ndarray) -> np.ndarray:
    """a_y = v^2 * kappa (planar curvature model)."""
    return (v ** 2) * kappa


def longitudinal_accel(v: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Finite-difference estimate of longitudinal acceleration a_x."""
    a = np.zeros_like(v)
    dt = np.diff(t)
    dv = np.diff(v)
    a[1:] = dv / dt
    a[0] = a[1]
    return a


def ltr_lateral(params: VehicleParams, a_y: np.ndarray) -> np.ndarray:
    """
    Lateral Load Transfer Ratio (LTR) for symmetric rigid body on flat ground:
        LTR_y = (Fz_R - Fz_L)/(Fz_R + Fz_L) = 2*h*a_y/(g*T)
    Tip-over threshold (quasi-static): |LTR_y| >= 1
    """
    return 2.0 * params.cg_height * a_y / (params.g * params.track)


def ltr_longitudinal(params: VehicleParams, a_x: np.ndarray) -> np.ndarray:
    """
    Longitudinal LTR (front-rear load transfer), analogous:
        LTR_x = (Fz_F - Fz_R)/(Fz_F + Fz_R) = 2*h*a_x/(g*L)
    with wheelbase L. Pitch tip-over (nose-over / tail-over) if |LTR_x| >= 1.
    """
    return 2.0 * params.cg_height * a_x / (params.g * params.wheelbase)


def tip_threshold_lateral(params: VehicleParams) -> float:
    """Quasi-static lateral tipping threshold for |a_y|."""
    return params.g * (params.track / (2.0 * params.cg_height))


def tip_threshold_longitudinal(params: VehicleParams) -> float:
    """Quasi-static longitudinal tipping threshold for |a_x|."""
    return params.g * (params.wheelbase / (2.0 * params.cg_height))


def slide_threshold(params: VehicleParams) -> float:
    """Sliding threshold for |a| using Coulomb friction (simplified): |a| <= mu*g."""
    return params.mu * params.g


def safe_speed_bounds(params: VehicleParams, kappa: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Conservative speed bounds along a path given curvature kappa:
    - Tipping-limited speed from |v|^2 |kappa| <= a_tip
    - Sliding-limited speed from |v|^2 |kappa| <= mu*g

    Returns: (v_tip_max, v_slide_max)
    """
    k = np.maximum(np.abs(kappa), 1e-9)
    a_tip = tip_threshold_lateral(params)
    v_tip = np.sqrt(a_tip / k)
    v_sl = np.sqrt(slide_threshold(params) / k)
    return v_tip, v_sl


def risk_metrics(params: VehicleParams, t: np.ndarray, v: np.ndarray, kappa: np.ndarray) -> dict:
    """
    Compute stability metrics along a trajectory.

    Returns dict with:
      a_y, a_x, LTR_y, LTR_x, margin_tip_y, margin_tip_x, margin_slide
    """
    a_y = lateral_accel(v, kappa)
    a_x = longitudinal_accel(v, t)

    ltry = ltr_lateral(params, a_y)
    ltrx = ltr_longitudinal(params, a_x)

    # Margins: positive is safe (conservative), negative indicates violation.
    margin_tip_y = 1.0 - np.abs(ltry)
    margin_tip_x = 1.0 - np.abs(ltrx)

    # Sliding margin based on combined planar acceleration magnitude (simplified).
    a_planar = np.sqrt(a_x ** 2 + a_y ** 2)
    margin_slide = slide_threshold(params) - a_planar

    return {
        "a_y": a_y, "a_x": a_x,
        "LTR_y": ltry, "LTR_x": ltrx,
        "margin_tip_y": margin_tip_y,
        "margin_tip_x": margin_tip_x,
        "margin_slide": margin_slide,
    }


def demo() -> None:
    """
    Demonstration on a synthetic path: a gentle-to-tight turn with a speed profile.
    """
    params = VehicleParams(track=0.55, wheelbase=0.65, cg_height=0.25, mu=0.7)

    t = np.linspace(0.0, 12.0, 601)
    # Speed ramps up then down [m/s]
    v = 0.2 + 1.4 * (1 - np.exp(-t / 2.5)) * (1 - 0.4 * (t > 8.0) * (t - 8.0) / 4.0)
    v = np.clip(v, 0.0, 1.8)

    # Curvature profile [1/m] (0 -> 0.8, then relax)
    kappa = 0.05 + 0.35 * np.exp(-((t - 6.0) / 2.0) ** 2)

    m = risk_metrics(params, t, v, kappa)
    v_tip, v_sl = safe_speed_bounds(params, kappa)

    print("=== Vehicle params ===")
    print(params)
    print("a_tip_y [m/s^2] =", tip_threshold_lateral(params))
    print("mu*g [m/s^2]    =", slide_threshold(params))
    print("Min margin_tip_y =", float(np.min(m["margin_tip_y"])))
    print("Min margin_slide =", float(np.min(m["margin_slide"])))

    # Optional plotting
    try:
        import matplotlib.pyplot as plt

        plt.figure()
        plt.plot(t, v, label="v(t)")
        plt.plot(t, np.minimum(v_tip, v_sl), label="min(v_tip, v_slide)")
        plt.xlabel("t [s]")
        plt.ylabel("speed [m/s]")
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(t, m["LTR_y"], label="LTR_y")
        plt.plot(t, np.sign(m["LTR_y"]), "--", label="tip threshold (+/-1)")
        plt.xlabel("t [s]")
        plt.ylabel("LTR_y [-]")
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(t, m["margin_tip_y"], label="margin_tip_y = 1 - |LTR_y|")
        plt.plot(t, m["margin_slide"], label="margin_slide = mu*g - ||a||")
        plt.axhline(0.0, linestyle="--")
        plt.xlabel("t [s]")
        plt.ylabel("margin")
        plt.legend()
        plt.grid(True)

        plt.show()
    except Exception as e:
        print("Plotting skipped:", e)


if __name__ == "__main__":
    demo()
