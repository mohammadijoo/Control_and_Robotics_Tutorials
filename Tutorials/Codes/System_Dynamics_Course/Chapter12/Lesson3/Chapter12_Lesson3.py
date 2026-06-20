"""
Chapter12_Lesson3.py
System Dynamics — Chapter 12, Lesson 3
Resonance, Bandwidth, and Quality Factor in Mechanical and Electrical Systems

Dependencies (recommended):
  - numpy
  - scipy
Optional:
  - control  (python-control)
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional, Tuple, Sequence

import numpy as np

try:
    from scipy import signal
except Exception as e:
    signal = None


@dataclass(frozen=True)
class SecondOrderParams:
    wn: float     # natural frequency (rad/s)
    zeta: float   # damping ratio
    Q: float      # quality factor (dimensionless)


def second_order_from_mck(m: float, c: float, k: float) -> SecondOrderParams:
    """
    Mass-spring-damper: m x¨ + c x˙ + k x = f
    Transfer from force to displacement: X/F = 1 / (m s^2 + c s + k)

    wn = sqrt(k/m)
    zeta = c / (2*sqrt(k*m))
    Q = 1/(2*zeta)   (valid for standard 2nd-order form)
    """
    if m <= 0 or k <= 0:
        raise ValueError("m and k must be positive.")
    wn = math.sqrt(k / m)
    zeta = c / (2.0 * math.sqrt(k * m))
    if zeta <= 0:
        Q = float("inf")
    else:
        Q = 1.0 / (2.0 * zeta)
    return SecondOrderParams(wn=wn, zeta=zeta, Q=Q)


def resonance_frequency(wn: float, zeta: float) -> Optional[float]:
    """
    Resonant frequency (magnitude peak) for the normalized low-pass 2nd-order:
        G(s) = wn^2 / (s^2 + 2 zeta wn s + wn^2)
    exists only if zeta < 1/sqrt(2)
        wr = wn * sqrt(1 - 2 zeta^2)
    """
    if zeta < 0:
        raise ValueError("zeta must be nonnegative.")
    if zeta >= 1.0 / math.sqrt(2.0):
        return None
    return wn * math.sqrt(1.0 - 2.0 * zeta * zeta)


def peak_magnitude_normalized_lowpass(zeta: float) -> Optional[float]:
    """
    Peak |G(jw)| for normalized low-pass second-order above.
    If zeta >= 1/sqrt(2), no interior peak; max occurs at w=0 and equals 1.
    If zeta < 1/sqrt(2):
        Mr = 1 / (2 zeta sqrt(1 - zeta^2))
    """
    if zeta >= 1.0 / math.sqrt(2.0):
        return None
    return 1.0 / (2.0 * zeta * math.sqrt(1.0 - zeta * zeta))


def half_power_frequencies_relative_to_peak(wn: float, zeta: float) -> Optional[Tuple[float, float]]:
    """
    Half-power frequencies w1 < w2 around the resonant peak of normalized low-pass.
    Defined by: |G(jw)|^2 = (1/2) |G(jwr)|^2

    Exact (in terms of r = w/wn):
        r1^2 = 1 - 2 zeta^2 - 2 zeta sqrt(1 - zeta^2)
        r2^2 = 1 - 2 zeta^2 + 2 zeta sqrt(1 - zeta^2)

    Requires zeta < 1/sqrt(2) (peak exists).
    """
    wr = resonance_frequency(wn, zeta)
    if wr is None:
        return None
    inside = 1.0 - zeta * zeta
    if inside <= 0:
        return None
    r1_sq = 1.0 - 2.0 * zeta * zeta - 2.0 * zeta * math.sqrt(inside)
    r2_sq = 1.0 - 2.0 * zeta * zeta + 2.0 * zeta * math.sqrt(inside)
    if r1_sq <= 0 or r2_sq <= 0:
        return None
    return (wn * math.sqrt(r1_sq), wn * math.sqrt(r2_sq))


def bandwidth_from_half_power(w1: float, w2: float) -> float:
    return float(w2 - w1)


def approximate_bandwidth_small_damping(wn: float, zeta: float) -> float:
    """
    For zeta << 1:
        Delta w ≈ 2 zeta wn
    """
    return 2.0 * zeta * wn


def estimate_peak_and_half_power_from_samples(w: Sequence[float], mag: Sequence[float]) -> dict:
    """
    Estimate resonant peak and half-power points from sampled magnitude data.

    Inputs:
      w   : frequencies in rad/s (monotone increasing)
      mag : |G(jw)| magnitudes at corresponding points

    Outputs:
      dict with keys: wr_hat, Mr_hat, w1_hat, w2_hat, bw_hat, Q_hat (if possible)
    """
    w = np.asarray(w, dtype=float)
    mag = np.asarray(mag, dtype=float)
    if w.ndim != 1 or mag.ndim != 1 or w.size != mag.size:
        raise ValueError("w and mag must be 1D arrays of same length.")
    if np.any(np.diff(w) <= 0):
        raise ValueError("w must be strictly increasing.")

    idx = int(np.argmax(mag))
    wr_hat = float(w[idx])
    Mr_hat = float(mag[idx])

    target = Mr_hat / math.sqrt(2.0)  # half-power magnitude relative to peak

    # Find left crossing
    w1_hat = None
    for i in range(idx, 0, -1):
        if (mag[i] - target) * (mag[i-1] - target) <= 0:
            # linear interpolation
            t = (target - mag[i-1]) / (mag[i] - mag[i-1] + 1e-300)
            w1_hat = float(w[i-1] + t * (w[i] - w[i-1]))
            break

    # Find right crossing
    w2_hat = None
    for i in range(idx, w.size - 1):
        if (mag[i] - target) * (mag[i+1] - target) <= 0:
            t = (target - mag[i]) / (mag[i+1] - mag[i] + 1e-300)
            w2_hat = float(w[i] + t * (w[i+1] - w[i]))
            break

    out = {"wr_hat": wr_hat, "Mr_hat": Mr_hat, "w1_hat": w1_hat, "w2_hat": w2_hat}
    if w1_hat is not None and w2_hat is not None and w2_hat > w1_hat:
        bw = w2_hat - w1_hat
        out["bw_hat"] = float(bw)
        out["Q_hat"] = float(wr_hat / bw) if bw > 0 else None
    else:
        out["bw_hat"] = None
        out["Q_hat"] = None
    return out


def demo_mass_spring_damper():
    # Example physical parameters (units: kg, N*s/m, N/m)
    m, c, k = 1.0, 0.4, 100.0
    p = second_order_from_mck(m, c, k)

    wr = resonance_frequency(p.wn, p.zeta)
    hw = half_power_frequencies_relative_to_peak(p.wn, p.zeta)
    print("Mass–spring–damper parameters:")
    print(f"  wn   = {p.wn:.6g} rad/s")
    print(f"  zeta = {p.zeta:.6g}")
    print(f"  Q    = {p.Q:.6g}")
    print(f"  wr   = {wr:.6g} rad/s" if wr else "  wr   = (no resonant peak)")
    if hw:
        w1, w2 = hw
        bw = bandwidth_from_half_power(w1, w2)
        print(f"  w1   = {w1:.6g} rad/s")
        print(f"  w2   = {w2:.6g} rad/s")
        print(f"  BW   = {bw:.6g} rad/s")
        print(f"  Q_hp = {wr/bw:.6g} (wr/BW)")
        print(f"  BW approx (2 zeta wn) = {approximate_bandwidth_small_damping(p.wn, p.zeta):.6g} rad/s")

    # Numerical frequency response check
    if signal is None:
        print("\nscipy is not available; skipping numerical frequency response demo.")
        return

    # Force -> displacement transfer: G(s) = 1/(m s^2 + c s + k)
    sys = signal.TransferFunction([1.0], [m, c, k])
    w = np.logspace(-1, 3, 4000)  # rad/s
    w, H = signal.freqresp(sys, w=w)
    mag = np.abs(H)

    est = estimate_peak_and_half_power_from_samples(w, mag)
    print("\nEstimated from sampled frequency response:")
    for k, v in est.items():
        print(f"  {k}: {v}")


def demo_series_rlc():
    """
    Series RLC band-pass across R:
      Z = R + j(wL - 1/(wC))
      I = V / Z
      Vr = I*R -> |Vr/V| = R/|Z|

    Resonance at w0 = 1/sqrt(LC)
    Quality factor (series): Q = w0 L / R = 1/(w0 C R)
    Bandwidth (rad/s): Delta w = R/L
    """
    R, L, C = 10.0, 50e-3, 10e-6
    w0 = 1.0 / math.sqrt(L * C)
    Q = w0 * L / R
    bw = R / L
    print("\nSeries RLC example:")
    print(f"  w0 = {w0:.6g} rad/s")
    print(f"  Q  = {Q:.6g}")
    print(f"  BW = {bw:.6g} rad/s")


if __name__ == "__main__":
    demo_mass_spring_damper()
    demo_series_rlc()
