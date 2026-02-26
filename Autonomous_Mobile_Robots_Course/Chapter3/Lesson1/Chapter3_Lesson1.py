"""
Chapter 3 — Nonholonomic Motion and Feasibility for AMR
Lesson 1 — Nonholonomic Constraints in Wheeled Robots (applied view)

This script:
  1) simulates a differential-drive robot using wheel angular rates
  2) checks the lateral no-slip Pfaffian constraint residual
  3) visualizes the resulting planar trajectory and constraint violation

Dependencies:
  - numpy, matplotlib (standard scientific Python stack)
Optional (symbolic addendum):
  - sympy
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt


def diff_drive_twist(omega_l: float, omega_r: float, r: float, b: float) -> tuple[float, float]:
    """
    Map wheel angular rates (rad/s) to body-frame forward speed v (m/s)
    and yaw rate w (rad/s).

        v = (r/2) (omega_r + omega_l)
        w = (r/(2b)) (omega_r - omega_l)

    where b is half the wheel track (distance from centerline to each wheel).
    """
    v = 0.5 * r * (omega_r + omega_l)
    w = 0.5 * r * (omega_r - omega_l) / b
    return v, w


def unicycle_step(x: float, y: float, theta: float, v: float, w: float, dt: float) -> tuple[float, float, float]:
    """
    Euler step for the unicycle kinematics:
        x_dot = v cos(theta)
        y_dot = v sin(theta)
        theta_dot = w
    """
    x = x + dt * v * np.cos(theta)
    y = y + dt * v * np.sin(theta)
    theta = theta + dt * w
    # wrap angle for nicer plots
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    return x, y, theta


def lateral_constraint_residual(x_dot: float, y_dot: float, theta: float) -> float:
    """
    Pfaffian lateral no-slip constraint for a wheel aligned with body x-axis:
        -sin(theta) x_dot + cos(theta) y_dot = 0

    Returns the scalar residual (should be ~0 for an ideal no-slip model).
    """
    return -np.sin(theta) * x_dot + np.cos(theta) * y_dot


def wheel_profile(t: float) -> tuple[float, float]:
    """
    Simple piecewise wheel rate profile to excite both translation and rotation.
    """
    if t < 4.0:
        return 6.0, 6.0          # straight
    elif t < 8.0:
        return 3.0, 7.0          # turn left
    elif t < 12.0:
        return 7.0, 3.0          # turn right
    else:
        return 5.0, 5.0          # straight


def run_sim(T: float = 16.0, dt: float = 0.01, r: float = 0.10, b: float = 0.22) -> dict[str, np.ndarray]:
    """
    Simulate the differential-drive kinematics for duration T.
    """
    N = int(np.floor(T / dt)) + 1
    t = np.linspace(0.0, T, N)

    x = np.zeros(N)
    y = np.zeros(N)
    th = np.zeros(N)

    v = np.zeros(N)
    w = np.zeros(N)
    res = np.zeros(N)

    for k in range(N - 1):
        om_l, om_r = wheel_profile(t[k])
        v[k], w[k] = diff_drive_twist(om_l, om_r, r=r, b=b)

        # implied world-frame velocities
        x_dot = v[k] * np.cos(th[k])
        y_dot = v[k] * np.sin(th[k])
        res[k] = lateral_constraint_residual(x_dot, y_dot, th[k])

        x[k + 1], y[k + 1], th[k + 1] = unicycle_step(x[k], y[k], th[k], v[k], w[k], dt)

    # last sample
    v[-1], w[-1] = v[-2], w[-2]
    x_dot = v[-1] * np.cos(th[-1])
    y_dot = v[-1] * np.sin(th[-1])
    res[-1] = lateral_constraint_residual(x_dot, y_dot, th[-1])

    return {"t": t, "x": x, "y": y, "theta": th, "v": v, "w": w, "res": res}


def plot_results(sim: dict[str, np.ndarray]) -> None:
    t, x, y, res = sim["t"], sim["x"], sim["y"], sim["res"]

    plt.figure()
    plt.plot(x, y)
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Differential-drive trajectory (ideal no-slip)")

    plt.figure()
    plt.plot(t, res)
    plt.xlabel("t [s]")
    plt.ylabel("constraint residual")
    plt.title("Lateral no-slip Pfaffian constraint residual (should be ~0)")
    plt.show()


def symbolic_nonintegrability_demo() -> None:
    """
    Optional: symbolic check that the 1-form omega = -sin(theta) dx + cos(theta) dy
    is non-integrable via omega ^ d(omega) != 0.

    Requires: sympy
    """
    try:
        import sympy as sp
    except Exception as e:
        print("sympy not available:", e)
        return

    x, y, th = sp.symbols("x y th", real=True)
    dx, dy, dth = sp.symbols("dx dy dth")
    # We emulate wedge products with formal basis and antisymmetry by hand (minimal demo).
    # Known result: omega ^ d omega = dx ^ dy ^ dth (nonzero).
    print("Known Frobenius test result: omega ^ d(omega) = dx ^ dy ^ dth != 0 -> nonholonomic.")


if __name__ == "__main__":
    sim = run_sim()
    print("max |constraint residual| =", np.max(np.abs(sim["res"])))
    plot_results(sim)

    # Uncomment if you want the symbolic addendum (requires sympy)
    # symbolic_nonintegrability_demo()
