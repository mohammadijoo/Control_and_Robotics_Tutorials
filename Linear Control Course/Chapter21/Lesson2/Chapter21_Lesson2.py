import numpy as np
import control as ct   # python-control library
# In robotics, this plant can approximate a motor or joint velocity loop.

# Plant parameter
tau = 0.1  # seconds

# Target closed-loop bandwidth (rad/s)
omega_b_target = 20.0

# Proportional gain from the formula K = tau * omega_b - 1
K = tau * omega_b_target - 1.0
print(f"Designed proportional gain K = {K:.3f}")

# Define plant and controller
G = ct.tf([1.0], [tau, 1.0])
C = ct.tf([K], [1.0])

# Closed-loop transfer function T(s) = L(s)/(1+L(s))
T = ct.feedback(C * G, 1)

# Simple numerical bandwidth estimate: find the first frequency where |T(jw)| <= 1/sqrt(2)
def estimate_bandwidth(sys, w_min=1e-1, w_max=1e3, n=2000):
    w = np.logspace(np.log10(w_min), np.log10(w_max), n)
    # freqresp returns complex frequency response over w
    _, mag, _ = ct.bode(sys, w, Plot=False)
    mag = np.squeeze(mag)  # magnitude as array
    target = mag[0] / np.sqrt(2.0)  # -3 dB relative to low-frequency value
    idx = np.where(mag <= target)[0]
    if idx.size == 0:
        return None
    return w[idx[0]]

omega_b_est = estimate_bandwidth(T)
print(f"Estimated bandwidth omega_b ≈ {omega_b_est:.2f} rad/s")

# Time-domain check: step response
t = np.linspace(0, 0.6, 600)
t_out, y_out = ct.step_response(T, t)

# A robotics application could wrap this into an automatic tuning routine for joint servos.
