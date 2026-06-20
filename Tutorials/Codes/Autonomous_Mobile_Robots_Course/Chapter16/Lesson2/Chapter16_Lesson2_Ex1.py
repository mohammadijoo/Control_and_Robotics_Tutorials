"""
Chapter16_Lesson2_Ex1.py
Exercise: Single agent + single moving obstacle — sample a safe velocity
and visualize unsafe velocities (in velocity space) under a finite horizon.
"""
from __future__ import annotations
import math
import random
import numpy as np

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None


def norm(x):
    return float(np.linalg.norm(x))


def time_to_collision(p_rel, v_rel, R, T):
    # Quadratic intersection with ||p_rel + t v_rel|| = R
    if norm(p_rel) <= R:
        return True, 0.0
    a = float(v_rel @ v_rel)
    b = 2.0 * float(p_rel @ v_rel)
    c = float(p_rel @ p_rel) - R * R
    if a < 1e-12:
        return False, float("inf")
    disc = b * b - 4.0 * a * c
    if disc < 0:
        return False, float("inf")
    t = (-b - math.sqrt(disc)) / (2.0 * a)
    return (0.0 <= t <= T), t


def sample_disk(v_max, n):
    out = []
    for _ in range(n):
        ang = random.random() * 2.0 * math.pi
        r = math.sqrt(random.random()) * v_max
        out.append(np.array([r*math.cos(ang), r*math.sin(ang)], dtype=float))
    return out


def main():
    random.seed(0)
    np.random.seed(0)

    # Agent and obstacle configuration
    p_a = np.array([0.0, 0.0])
    v_a = np.array([0.0, 0.0])
    p_b = np.array([2.0, 0.8])
    v_b = np.array([-0.4, -0.05])

    r_a, r_b = 0.35, 0.35
    R = r_a + r_b
    T = 3.0
    v_max = 1.2

    # Preferred velocity (toward a goal)
    goal = np.array([4.0, 0.0])
    v_pref = (goal - p_a)
    v_pref = v_pref / (norm(v_pref) + 1e-12) * 1.0

    candidates = sample_disk(v_max, 4000)
    safe = []
    unsafe = []
    for v in candidates:
        coll, t = time_to_collision(p_b - p_a, v - v_b, R, T)
        if coll:
            unsafe.append(v)
        else:
            safe.append(v)

    # Choose the safe velocity closest to v_pref
    safe = np.array(safe) if safe else np.zeros((0,2))
    unsafe = np.array(unsafe) if unsafe else np.zeros((0,2))
    if safe.shape[0] > 0:
        idx = np.argmin(np.sum((safe - v_pref[None,:])**2, axis=1))
        v_star = safe[idx]
    else:
        v_star = np.zeros(2)

    print("Preferred v:", v_pref)
    print("Chosen safe v*:", v_star)

    if plt is None:
        print("matplotlib not available; skipping plot.")
        return

    plt.figure(figsize=(7,6))
    if unsafe.shape[0] > 0:
        plt.scatter(unsafe[:,0], unsafe[:,1], s=5, alpha=0.15, label="Unsafe (inside VO_T)")
    if safe.shape[0] > 0:
        plt.scatter(safe[:,0], safe[:,1], s=6, alpha=0.25, label="Safe")
    plt.scatter([v_pref[0]], [v_pref[1]], marker="x", s=120, label="v_pref")
    plt.scatter([v_star[0]], [v_star[1]], marker="o", s=120, label="v*")
    plt.axis("equal")
    plt.xlabel("v_x")
    plt.ylabel("v_y")
    plt.title("Velocity-space safe/unsafe samples (finite horizon)")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
