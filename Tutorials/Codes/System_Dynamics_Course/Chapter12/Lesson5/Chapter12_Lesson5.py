#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Chapter 12 - Lesson 5: Time–Frequency Domain Relationships and Trade-offs
# File: Chapter12_Lesson5.py
#
# This script demonstrates quantitative links between time-domain metrics
# (rise time, settling time, overshoot) and frequency-domain metrics
# (bandwidth, resonant peak) for the canonical 2nd-order low-pass system:
#     G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
#
# Dependencies:
#   - numpy
#   - scipy
#
# Optional:
#   - matplotlib (for plots)

import numpy as np
from scipy import signal


def second_order_tf(zeta: float, wn: float) -> signal.TransferFunction:
    num = [wn**2]
    den = [1.0, 2.0 * zeta * wn, wn**2]
    return signal.TransferFunction(num, den)


def step_metrics(t: np.ndarray, y: np.ndarray, tol: float = 0.02):
    """Compute basic step metrics for a unit-step response."""
    y_final = float(y[-1])
    y_peak = float(np.max(y))
    Mp = 0.0 if y_final == 0.0 else max(0.0, (y_peak - y_final) / abs(y_final)) * 100.0

    # Rise time: first crossing of 10% and 90% of final value
    y10, y90 = 0.1 * y_final, 0.9 * y_final

    def first_cross(level: float):
        idx = np.where(y >= level)[0]
        return float(t[idx[0]]) if len(idx) else float("nan")

    tr = first_cross(y90) - first_cross(y10)

    # Settling time: last time outside tolerance band
    band_low, band_high = (1.0 - tol) * y_final, (1.0 + tol) * y_final
    outside = np.where((y < band_low) | (y > band_high))[0]
    ts = float(t[outside[-1]]) if len(outside) else 0.0

    return {"y_final": y_final, "Mp_percent": float(Mp), "tr": float(tr), "ts": float(ts)}


def frequency_metrics(sys: signal.TransferFunction, w: np.ndarray):
    """Compute resonant peak, resonant frequency, and -3 dB bandwidth."""
    w, H = signal.freqresp(sys, w=w)
    mag = np.abs(H)

    Mr = float(np.max(mag))
    wr = float(w[int(np.argmax(mag))])

    dc = float(mag[0])  # for this low-pass, mag[0] ~ 1
    target = dc / np.sqrt(2.0)
    idx = np.where(mag <= target)[0]
    wb = float(w[idx[0]]) if len(idx) else float("nan")

    return {"Mr": Mr, "wr": wr, "wb": wb}


def demo_single(zeta=0.3, wn=10.0):
    sys = second_order_tf(float(zeta), float(wn))

    # Time response
    t = np.linspace(0, 3.0, 4000)
    t, y = signal.step(sys, T=t)
    tm = step_metrics(t, y)

    # Frequency response
    w = np.logspace(-2, 3, 2000) * wn
    fm = frequency_metrics(sys, w)

    return tm, fm


def sweep_zeta(wn=10.0, zetas=(0.1, 0.2, 0.3, 0.5, 0.7)):
    results = []
    for z in zetas:
        tm, fm = demo_single(zeta=float(z), wn=float(wn))
        results.append((float(z), tm["Mp_percent"], tm["tr"], tm["ts"], fm["Mr"], fm["wb"]))
    return results


if __name__ == "__main__":
    zeta, wn = 0.35, 12.0
    tm, fm = demo_single(zeta=zeta, wn=wn)

    print("=== Single system ===")
    print(f"zeta={zeta:.3f}, wn={wn:.3f} rad/s")
    print(f"Overshoot Mp = {tm['Mp_percent']:.2f}%")
    print(f"Rise time tr = {tm['tr']:.4f} s (10%->90%)")
    print(f"Settling time ts = {tm['ts']:.4f} s (2%)")
    print(f"Resonant peak Mr = {fm['Mr']:.4f}")
    print(f"Bandwidth wb = {fm['wb']:.4f} rad/s")

    print("\n=== Sweep zeta (fixed wn) ===")
    rows = sweep_zeta(wn=wn, zetas=(0.15, 0.25, 0.35, 0.50, 0.70))
    print("zeta, Mp(%), tr(s), ts(s), Mr, wb(rad/s)")
    for r in rows:
        print("{:.2f}, {:6.2f}, {:7.4f}, {:7.4f}, {:7.4f}, {:9.4f}".format(*r))

    # Optional plotting (skipped if matplotlib is not available)
    try:
        import matplotlib.pyplot as plt

        zetas = np.linspace(0.15, 0.9, 20)
        Mp_list, Mr_list, wb_list = [], [], []
        for z in zetas:
            tm_i, fm_i = demo_single(zeta=float(z), wn=wn)
            Mp_list.append(tm_i["Mp_percent"])
            Mr_list.append(fm_i["Mr"])
            wb_list.append(fm_i["wb"])

        plt.figure()
        plt.plot(Mp_list, Mr_list)
        plt.xlabel("Overshoot Mp (%)")
        plt.ylabel("Resonant peak Mr")
        plt.title("Overshoot vs Resonant Peak (vary zeta)")
        plt.grid(True)

        plt.figure()
        plt.plot(zetas, wb_list)
        plt.xlabel("zeta")
        plt.ylabel("Bandwidth wb (rad/s)")
        plt.title("Bandwidth vs Damping Ratio (fixed wn)")
        plt.grid(True)

        plt.show()
    except Exception as e:
        print("\nPlotting skipped:", e)
