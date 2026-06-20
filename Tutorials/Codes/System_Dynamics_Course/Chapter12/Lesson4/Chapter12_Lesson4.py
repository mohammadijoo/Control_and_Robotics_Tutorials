# Chapter12_Lesson4.py
# Nyquist and Nichols Plots (Introductory Level) — Frequency Response and Resonance
#
# Dependencies:
#   pip install control numpy matplotlib
#
# This script:
#   1) Defines an LTI transfer function with a lightly-damped resonant mode.
#   2) Computes and plots Nyquist and Nichols curves from sampled frequency response.
#   3) Computes classical stability margins (gain/phase).
#
# Output files:
#   - Chapter12_Lesson4_nyquist.png
#   - Chapter12_Lesson4_nichols.png
#   - Chapter12_Lesson4_freqresp.csv

import numpy as np
import matplotlib.pyplot as plt

try:
    import control  # python-control
except ImportError as e:
    raise SystemExit("Missing dependency 'control'. Install with: pip install control") from e


def poly_eval_complex(coeffs, s):
    """Evaluate polynomial with real coefficients at complex s (highest power first)."""
    y = 0.0 + 0.0j
    for c in coeffs:
        y = y * s + c
    return y


def tf_freqresp_manual(num, den, w):
    """Manual complex frequency response G(jw) for transfer function num(s)/den(s)."""
    Gjw = np.zeros_like(w, dtype=np.complex128)
    for k, wk in enumerate(w):
        s = 1j * wk
        Gjw[k] = poly_eval_complex(num, s) / poly_eval_complex(den, s)
    return Gjw


# Example open-loop plant with a resonant mode and a first-order pole.
# G(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * 1/(tau*s + 1)
K = 5.0
wn = 10.0
zeta = 0.20
tau = 0.05

# Transfer function construction (control.tf wants numerator/denominator polynomials)
# First: resonant second-order: wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
num1 = [wn**2]
den1 = [1.0, 2*zeta*wn, wn**2]

# Second: first-order pole: 1/(tau*s + 1)
num2 = [1.0]
den2 = [tau, 1.0]

sys = K * control.tf(num1, den1) * control.tf(num2, den2)

# Frequency grid
w = np.logspace(-1, 2.5, 1500)  # rad/s

# Frequency response via library and manual check
Gjw = np.array([control.evalfr(sys, 1j*wk) for wk in w], dtype=np.complex128)

# Manual using combined polynomial (optional cross-check)
num = np.polymul(np.array(num1), np.array(num2)) * K
den = np.polymul(np.array(den1), np.array(den2))
Gjw_manual = tf_freqresp_manual(num, den, w)

max_err = np.max(np.abs(Gjw - Gjw_manual))
print("Max |Gjw - Gjw_manual| =", max_err)

# Export frequency response
mag = np.abs(Gjw)
phase = np.angle(Gjw, deg=True)
mag_db = 20*np.log10(mag)
out = np.column_stack([w, Gjw.real, Gjw.imag, mag_db, phase])
np.savetxt("Chapter12_Lesson4_freqresp.csv", out,
           delimiter=",",
           header="omega_rad_s,ReG,ImG,Mag_dB,Phase_deg",
           comments="")
print("Wrote Chapter12_Lesson4_freqresp.csv")

# Nyquist plot (parametric plot of G(jw))
plt.figure()
plt.plot(Gjw.real, Gjw.imag, linewidth=1.0, label="G(jw), w>0")
# Conjugate symmetry for real-coefficient systems:
plt.plot(Gjw.real, -Gjw.imag, linewidth=1.0, label="G(jw), w<0 (symmetry)")
plt.scatter([-1.0], [0.0], s=35, marker="x", label="-1 point")
plt.axhline(0.0, linewidth=0.8)
plt.axvline(0.0, linewidth=0.8)
plt.gca().set_aspect("equal", adjustable="box")
plt.xlabel("Re{G(jw)}")
plt.ylabel("Im{G(jw)}")
plt.title("Nyquist Curve")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("Chapter12_Lesson4_nyquist.png", dpi=200)
print("Saved Chapter12_Lesson4_nyquist.png")

# Nichols plot: magnitude (dB) vs phase (deg)
phase_unwrapped = np.unwrap(np.deg2rad(phase))
phase_unwrapped_deg = np.rad2deg(phase_unwrapped)

plt.figure()
plt.plot(phase_unwrapped_deg, mag_db, linewidth=1.0)
plt.xlabel("Phase (deg)")
plt.ylabel("Magnitude (dB)")
plt.title("Nichols Curve")
plt.grid(True)
plt.tight_layout()
plt.savefig("Chapter12_Lesson4_nichols.png", dpi=200)
print("Saved Chapter12_Lesson4_nichols.png")

# Classical margins (unity feedback) from open-loop L(s)=G(s)
gm, pm, wgc, wpc = control.margin(sys)
print("Gain margin (abs) =", gm)
print("Phase margin (deg) =", pm)
print("Gain crossover w_gc (rad/s) =", wgc)
print("Phase crossover w_pc (rad/s) =", wpc)

# Closed-loop check: T(s)=G/(1+G)
T = control.feedback(sys, 1)
Tjw = np.array([control.evalfr(T, 1j*wk) for wk in w], dtype=np.complex128)
peak_T = np.max(np.abs(Tjw))
w_peak = w[np.argmax(np.abs(Tjw))]
print("Peak |T(jw)| ≈", peak_T, "at w ≈", w_peak, "rad/s")
