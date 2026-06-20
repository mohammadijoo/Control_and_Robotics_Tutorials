# Chapter15_Lesson4.py
"""
Event handling, switching, and hybrid continuous-discrete dynamics
Example: Bouncing ball with restitution using solve_ivp event detection.
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


g = 9.81           # gravitational acceleration (m/s^2)
e = 0.82           # coefficient of restitution
t0, tf = 0.0, 8.0  # simulation horizon
y0 = np.array([1.5, 0.0])  # [height, velocity]
min_bounce_speed = 0.05     # terminate when impacts become negligible


def flow(t, y):
    h, v = y
    return [v, -g]


def ground_event(t, y):
    # Event function g(y) = h. Trigger at h = 0 while descending.
    return y[0]


ground_event.terminal = True
ground_event.direction = -1.0


def jump_map(y_minus):
    h_minus, v_minus = y_minus
    # Reset state at impact: h^+ = 0, v^+ = -e v^-
    return np.array([0.0, -e * v_minus])


def simulate_hybrid_bouncing_ball():
    t_segments = []
    y_segments = []
    impact_times = []

    t_curr = t0
    y_curr = y0.copy()

    while t_curr < tf:
        sol = solve_ivp(
            flow,
            (t_curr, tf),
            y_curr,
            events=ground_event,
            dense_output=True,
            max_step=0.03,
            rtol=1e-8,
            atol=1e-10,
        )

        t_segments.append(sol.t)
        y_segments.append(sol.y.T)

        if sol.t_events[0].size == 0:
            break

        te = float(sol.t_events[0][0])
        ye_minus = sol.sol(te)
        impact_times.append(te)

        if abs(ye_minus[1]) < min_bounce_speed:
            # Stop after final small impact.
            t_segments.append(np.array([te]))
            y_segments.append(np.array([[0.0, 0.0]]))
            break

        y_plus = jump_map(ye_minus)
        t_curr = te
        # Restart slightly after the event to avoid immediate re-detection.
        y_curr = y_plus + np.array([0.0, 1e-12])

    return t_segments, y_segments, impact_times


if __name__ == "__main__":
    ts, ys, impacts = simulate_hybrid_bouncing_ball()

    t_plot = np.concatenate(ts)
    y_plot = np.vstack(ys)

    plt.figure(figsize=(8, 4))
    plt.plot(t_plot, y_plot[:, 0], label="height h(t)")
    for te in impacts:
        plt.axvline(te, linestyle="--", linewidth=0.8)
    plt.xlabel("Time (s)")
    plt.ylabel("Height (m)")
    plt.title("Hybrid Simulation: Bouncing Ball with Events")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    print("Impact times:", impacts)
