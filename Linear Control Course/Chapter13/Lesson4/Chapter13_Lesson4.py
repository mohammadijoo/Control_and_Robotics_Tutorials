import numpy as np
import control  # python-control: pip install control

# Example: design values for a robotic joint position loop
zeta = 0.6
wn = 13.5  # rad/s, roughly consistent with Section 6

# Closed-loop second-order transfer function T(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
num = [wn**2]
den = [1, 2*zeta*wn, wn**2]
T = control.TransferFunction(num, den)

# --- Time-domain analysis: step response ---
t = np.linspace(0, 2.0, 1000)  # 2 seconds horizon
t, y = control.step_response(T, T=t)

# Estimate overshoot Mp and 2% settling time ts
y_final = y[-1]
peak = np.max(y)
Mp = (peak - y_final) / y_final
# Settling time: last time y is outside 2% band
idx_outside = np.where(np.abs(y - y_final) > 0.02 * np.abs(y_final))[0]
ts = t[idx_outside[-1]] if idx_outside.size > 0 else 0.0

print(f"Estimated Mp: {Mp*100:.1f}%")
print(f"Estimated ts: {ts:.3f} s")

# --- Frequency-domain analysis ---
# Frequency range around the expected bandwidth
w = np.logspace(-1, 2, 500)  # 0.1 to 100 rad/s
mag, phase, omega = control.freqresp(T, w)

# mag has shape (1,1,len(w)) for SISO, take absolute and flatten
mag = np.abs(mag.flatten())

# Resonant peak and frequency
idx_Mr = np.argmax(mag)
Mr = mag[idx_Mr]
wr = omega[idx_Mr]

# Bandwidth: frequency where |T(jw)| drops to 1/sqrt(2) of DC (assume DC gain ~1)
target = 1 / np.sqrt(2)
# Find first index where magnitude falls below target
bw_indices = np.where(mag <= target)[0]
wB = omega[bw_indices[0]] if bw_indices.size > 0 else np.nan

print(f"Mr: {Mr:.3f}, wr: {wr:.3f} rad/s, wB: {wB:.3f} rad/s")
