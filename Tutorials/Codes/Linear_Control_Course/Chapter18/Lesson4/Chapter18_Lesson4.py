import numpy as np
import control as ctrl   # python-control library
from math import sqrt, pi

# Helper functions: zeta <-> overshoot, bandwidth, etc.
def zeta_from_Mp(Mp):
    """
    Invert Mp = exp(-pi*zeta / sqrt(1-zeta^2))
    using a simple numerical solve (Newton).
    Mp is specified as a fraction (e.g. 0.1 for 10%).
    """
    if Mp <= 0.0 or Mp >= 1.0:
        raise ValueError("Mp must be in (0,1)")
    z = 0.5  # initial guess
    for _ in range(20):
        f = np.exp(-pi*z / np.sqrt(1 - z**2)) - Mp
        df = np.exp(-pi*z / np.sqrt(1 - z**2)) * (
            (-pi / np.sqrt(1 - z**2)) +
            (-pi*z*(+z) / (1 - z**2)**(3/2))
        )
        z = z - f / df
        z = min(max(z, 1e-3), 0.999)
    return z

def Mr_from_zeta(zeta):
    if zeta >= 1/np.sqrt(2):
        return 1.0
    return 1.0 / (2*zeta*np.sqrt(1 - zeta**2))

def omega_n_from_ts(ts, zeta):
    return 4.0 / (zeta*ts)

def bandwidth_from_2nd_order(wn, zeta):
    # exact closed-form for 3 dB bandwidth
    term = np.sqrt(2 - 4*zeta**2 + 4*zeta**4)
    y = 1 - 2*zeta**2 + term
    return wn*np.sqrt(y)

# Desired time-domain specs for a robot joint servo
Mp_max = 0.1    # 10%
ts_max = 0.8    # seconds

zeta = zeta_from_Mp(Mp_max)
wn   = omega_n_from_ts(ts_max, zeta)
wB   = bandwidth_from_2nd_order(wn, zeta)
Mr   = Mr_from_zeta(zeta)

print("zeta ~", zeta)
print("wn   ~", wn)
print("omega_B ~", wB)
print("Mr   ~", Mr)

# Build an approximate closed-loop model (for verification)
T = ctrl.TransferFunction([wn**2], [1, 2*zeta*wn, wn**2])

t, y = ctrl.step_response(T)
info = ctrl.step_info(T)
print("Step info:", info)

# Bode magnitude at crossover near the bandwidth
mag, phase, omega = ctrl.bode(T, omega_limits=(0.1, 10*wB), Plot=False)
# Extract approximate -3 dB frequency
mag_db = 20*np.log10(mag)
idx = np.argmin(np.abs(mag_db + 3.0))
print("Approx -3 dB bandwidth from numeric Bode:", omega[idx])
