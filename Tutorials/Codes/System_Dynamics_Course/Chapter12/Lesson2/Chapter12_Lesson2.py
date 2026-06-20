"""
Chapter 12 - Lesson 2: Bode Plots: Magnitude, Phase, Asymptotes, and Construction Rules
System Dynamics (Control Engineering)

This script demonstrates:
1) Library-based Bode plotting using python-control.
2) Manual computation of frequency response G(jw), magnitude (dB), phase (deg).
3) Construction of simple asymptotic Bode magnitude and phase approximations
   for first-order poles/zeros and integrators.

Dependencies (pip):
  - numpy
  - matplotlib
  - control  (python-control)

Optional:
  - scipy (for unwrap; we also implement unwrap manually)
"""

from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt

try:
    import control  # type: ignore
except Exception as e:
    control = None
    print("python-control is not available:", e)


def polyval(coeffs: np.ndarray, s: np.ndarray) -> np.ndarray:
    """Evaluate polynomial with coefficients in descending powers."""
    y = np.zeros_like(s, dtype=np.complex128)
    for c in coeffs:
        y = y * s + c
    return y


def freq_response(num: np.ndarray, den: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Compute G(jw) for a rational transfer function with real coefficients."""
    s = 1j * w
    return polyval(num, s) / polyval(den, s)


def mag_db(Gjw: np.ndarray) -> np.ndarray:
    return 20.0 * np.log10(np.abs(Gjw))


def unwrap_phase_deg(phase_deg: np.ndarray) -> np.ndarray:
    """Simple phase unwrap in degrees (adds/subtracts 360 to reduce jumps)."""
    out = phase_deg.copy()
    for k in range(1, len(out)):
        d = out[k] - out[k - 1]
        if d > 180.0:
            out[k:] -= 360.0
        elif d < -180.0:
            out[k:] += 360.0
    return out


def phase_deg(Gjw: np.ndarray) -> np.ndarray:
    ph = np.angle(Gjw, deg=True)
    return unwrap_phase_deg(ph)


# ---------- Asymptotic construction helpers (first-order factors) ----------

def asym_mag_db_first_order_zero(w: np.ndarray, wc: float) -> np.ndarray:
    """
    Asymptotic magnitude (dB) for factor (1 + s/wc):
      0 dB for w < wc
      +20 dB/dec for w > wc
    """
    y = np.zeros_like(w, dtype=float)
    mask = w >= wc
    y[mask] = 20.0 * np.log10(w[mask] / wc)
    return y


def asym_mag_db_first_order_pole(w: np.ndarray, wc: float) -> np.ndarray:
    """Asymptotic magnitude (dB) for factor 1/(1 + s/wc)."""
    return -asym_mag_db_first_order_zero(w, wc)


def asym_phase_deg_first_order_zero(w: np.ndarray, wc: float) -> np.ndarray:
    """
    Standard Bode phase approximation for (1 + s/wc):
      0 deg for w <= 0.1 wc
      linear ramp from 0 to +90 between 0.1 wc and 10 wc (slope +45 deg/dec)
      +90 deg for w >= 10 wc
    """
    y = np.zeros_like(w, dtype=float)
    w1, w2 = 0.1 * wc, 10.0 * wc
    # linear in log10(w)
    mid = (w > w1) & (w < w2)
    y[w >= w2] = 90.0
    y[mid] = 45.0 * (np.log10(w[mid] / w1))
    return y


def asym_phase_deg_first_order_pole(w: np.ndarray, wc: float) -> np.ndarray:
    return -asym_phase_deg_first_order_zero(w, wc)


def asym_mag_db_integrator(w: np.ndarray, m: int = 1) -> np.ndarray:
    """Asymptotic magnitude for 1/s^m: -20m dB/dec relative to w=1 rad/s."""
    return -20.0 * m * np.log10(w)


def asym_phase_deg_integrator(m: int = 1) -> float:
    return -90.0 * m


# ---------- Demonstration system ----------
# Example transfer function:
#   G(s) = 10 * (1 + s/1) / ( s * (1 + s/10) )
# Numerator: 10*(s + 1) -> [10, 10]
# Denominator: s*(s/10 + 1) = s*(0.1 s + 1) -> 0.1 s^2 + s -> [0.1, 1, 0]
num = np.array([10.0, 10.0])
den = np.array([0.1, 1.0, 0.0])

w = np.logspace(-2, 3, 2000)  # rad/s

Gjw = freq_response(num, den, w)
M = mag_db(Gjw)
P = phase_deg(Gjw)

# Asymptotes: gain 10 (20 dB), one zero at wc=1, one pole at wc=10, one integrator
Mag_asym = 20.0 * np.log10(10.0) \
    + asym_mag_db_first_order_zero(w, wc=1.0) \
    + asym_mag_db_first_order_pole(w, wc=10.0) \
    + asym_mag_db_integrator(w, m=1)

Phase_asym = 0.0 \
    + asym_phase_deg_first_order_zero(w, wc=1.0) \
    + asym_phase_deg_first_order_pole(w, wc=10.0) \
    + asym_phase_deg_integrator(m=1)

# ---------- Plot ----------
plt.figure()
plt.semilogx(w, M, label="Exact magnitude (dB)")
plt.semilogx(w, Mag_asym, "--", label="Asymptotic magnitude (dB)")
plt.xlabel("Frequency w [rad/s]")
plt.ylabel("Magnitude [dB]")
plt.grid(True, which="both")
plt.legend()

plt.figure()
plt.semilogx(w, P, label="Exact phase (deg)")
plt.semilogx(w, Phase_asym, "--", label="Asymptotic phase (deg)")
plt.xlabel("Frequency w [rad/s]")
plt.ylabel("Phase [deg]")
plt.grid(True, which="both")
plt.legend()

# Library-based Bode (if control is installed)
if control is not None:
    sys_tf = control.TransferFunction(num, den)
    plt.figure()
    control.bode_plot(sys_tf, w, dB=True, Hz=False, deg=True, plot=True)
    plt.suptitle("python-control bode_plot (library)")

plt.show()
