import numpy as np
import control as ct  # python-control library

# Example open-loop: L(s) = K / (s (T s + 1))
K = 10.0
T = 0.1
num = [K]
den = [T, 1.0, 0.0]  # s (T s + 1) = T s^2 + s

L = ct.TransferFunction(num, den)

# Frequency grid (logarithmic)
w = np.logspace(-1, 3, 2000)  # 0.1 rad/s to 1000 rad/s

# Frequency response
mag, phase, w = ct.freqresp(L, w)
mag = mag.flatten()
phase = phase.flatten()   # radians

# Helpers to approximate zero crossings of a sampled function
def zero_crossings(x):
    """Return indices i where x[i] and x[i+1] have opposite sign."""
    s = np.sign(x)
    return np.where(np.diff(s) != 0)[0]

def interp_root(w, x, i):
    """Linear interpolation of root between w[i], w[i+1]."""
    w1, w2 = w[i], w[i+1]
    x1, x2 = x[i], x[i+1]
    return w1 - x1 * (w2 - w1) / (x2 - x1)

# Gain crossover: |L(jw)| = 1
g = mag - 1.0
idx_gc = zero_crossings(g)
w_gc = None
if idx_gc.size > 0:
    i = idx_gc[0]
    w_gc = interp_root(w, g, i)

# Phase crossover: angle(L(jw)) = -pi (mod 2 pi)
# Wrap phase to (-pi, pi] and shift by +pi so zero crossings correspond to -pi
phase_wrapped = (phase + np.pi) % (2 * np.pi) - np.pi
h = phase_wrapped + np.pi    # h = 0 when phase_wrapped = -pi

idx_pc = zero_crossings(h)
w_pc = None
if idx_pc.size > 0:
    i = idx_pc[0]
    w_pc = interp_root(w, h, i)

print("Approximate gain crossover w_gc:", w_gc)
print("Approximate phase crossover w_pc:", w_pc)

# Gain and phase margins
if w_gc is not None:
    L_gc = ct.evalfr(L, 1j * w_gc)
    pm_rad = np.pi + np.angle(L_gc)
    print("Phase margin (deg) ~", pm_rad * 180.0 / np.pi)

if w_pc is not None:
    L_pc = ct.evalfr(L, 1j * w_pc)
    gm = 1.0 / abs(L_pc)
    print("Gain margin ~", gm, " (", 20 * np.log10(gm), " dB )")
