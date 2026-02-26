"""
Chapter15_Lesson2_Ex1.py
Exercise: Gain sweep for Stanley controller (RMS cross-track error vs gain k)
This script reuses a simplified Stanley implementation to sweep k and plot RMS(e_y).

Author: Abolfazl Mohammadijoo (educational example)
"""

from __future__ import annotations
import math
import numpy as np
import matplotlib.pyplot as plt


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def make_s_path(n: int = 400):
    x = np.linspace(0.0, 50.0, n)
    y = 2.5 * np.sin(0.18 * x) + 1.0 * np.sin(0.04 * x)
    return x, y


def nearest_segment(px, py, x_ref, y_ref):
    x1 = x_ref[:-1]; y1 = y_ref[:-1]
    x2 = x_ref[1:];  y2 = y_ref[1:]
    dx = x2 - x1; dy = y2 - y1
    denom = dx*dx + dy*dy
    denom = np.where(denom < 1e-12, 1e-12, denom)
    t = ((px - x1)*dx + (py - y1)*dy) / denom
    t = np.clip(t, 0.0, 1.0)
    projx = x1 + t*dx
    projy = y1 + t*dy
    d2 = (px - projx)**2 + (py - projy)**2
    i = int(np.argmin(d2))
    return i, float(projx[i]), float(projy[i])


def segment_heading(i, x_ref, y_ref):
    return math.atan2(float(y_ref[i+1] - y_ref[i]), float(x_ref[i+1] - x_ref[i]))


def simulate(k_gain: float, v: float = 6.0, v0: float = 0.5, L: float = 2.7, dt: float = 0.02, T: float = 18.0):
    x_ref, y_ref = make_s_path()
    steps = int(T/dt)
    x, y, psi = -2.0, 3.5, math.radians(-10)
    eys = []
    for _ in range(steps):
        i, nx, ny = nearest_segment(x, y, x_ref, y_ref)
        psi_ref = segment_heading(i, x_ref, y_ref)
        e_psi = wrap_angle(psi_ref - psi)
        tx, ty = math.cos(psi_ref), math.sin(psi_ref)
        vx, vy = x - nx, y - ny
        cross = tx*vy - ty*vx
        e_y = math.copysign(math.hypot(vx, vy), cross)
        delta = e_psi + math.atan2(k_gain * e_y, v + v0)
        delta = max(-math.radians(32), min(math.radians(32), delta))
        # bicycle step
        x += v * math.cos(psi) * dt
        y += v * math.sin(psi) * dt
        psi = wrap_angle(psi + v/L * math.tan(delta) * dt)
        eys.append(e_y)
    eys = np.asarray(eys)
    rms = float(np.sqrt(np.mean(eys**2)))
    return rms


def main():
    ks = np.linspace(0.2, 4.0, 20)
    rms = [simulate(k) for k in ks]

    plt.figure(figsize=(8, 4))
    plt.plot(ks, rms, marker="o")
    plt.grid(True)
    plt.xlabel("k (cross-track gain)")
    plt.ylabel("RMS(e_y) [m]")
    plt.title("Stanley gain sweep (educational)")
    plt.show()

    best = ks[int(np.argmin(rms))]
    print("Best k in sweep:", best)


if __name__ == "__main__":
    main()
