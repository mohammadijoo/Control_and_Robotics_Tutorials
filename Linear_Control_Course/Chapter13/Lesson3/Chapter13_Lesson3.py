import numpy as np
import matplotlib.pyplot as plt

# Control and robotics related libraries
# python-control: general LTI analysis
# roboticstoolbox: robot models whose joints are often approximated by 2nd-order systems
import control

# Closed-loop 2nd-order model parameters
wn = 10.0   # natural frequency [rad/s]
zeta = 0.3  # damping ratio

# Transfer function T(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
num = [wn**2]
den = [1.0, 2.0*zeta*wn, wn**2]
T = control.tf(num, den)

# Frequency grid for analysis
w = np.logspace(-1, 2, 2000)  # 0.1 to 100 rad/s

# Bode magnitude (no plotting for programmatic analysis)
mag, phase, w_out = control.bode(T, w, Plot=False)
mag = np.squeeze(mag)   # shape (N,)
phase = np.squeeze(phase)

# Resonant peak Mr and resonant frequency wr
idx_Mr = int(np.argmax(mag))
Mr_num = mag[idx_Mr]
wr_num = w_out[idx_Mr]

# -3 dB target magnitude relative to DC gain
mag0 = mag[0]  # should be ~1
target = mag0 / np.sqrt(2.0)

# Find first frequency where magnitude falls below target
wb_num = w_out[-1]
for wi, mi in zip(w_out, mag):
    if mi <= target:
        wb_num = wi
        break

print(f"Numeric resonant peak Mr  : {Mr_num:.3f}")
print(f"Numeric resonant freq wr  : {wr_num:.3f} rad/s")
print(f"Numeric bandwidth wb      : {wb_num:.3f} rad/s")

# Analytical expressions for comparison (valid for zeta < 1/sqrt(2))
if zeta < 1.0/np.sqrt(2.0):
    wr_analytic = wn * np.sqrt(1.0 - 2.0*zeta**2)
    Mr_analytic = 1.0 / (2.0*zeta*np.sqrt(1.0 - zeta**2))
    print(f"Analytic Mr               : {Mr_analytic:.3f}")
    print(f"Analytic wr               : {wr_analytic:.3f} rad/s")

# Optional: plot magnitude
plt.figure()
plt.semilogx(w_out, 20*np.log10(mag))
plt.axhline(20*np.log10(target), linestyle="--")
plt.axvline(wr_num, linestyle=":", label="wr")
plt.axvline(wb_num, linestyle="--", label="wb")
plt.xlabel("w [rad/s]")
plt.ylabel("Magnitude [dB]")
plt.legend()
plt.grid(True, which="both")
plt.title("Closed-loop 2nd-order magnitude")
plt.show()
