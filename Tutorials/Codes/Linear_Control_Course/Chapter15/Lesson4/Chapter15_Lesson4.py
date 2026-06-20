import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Unstable plant with RHP pole at p > 0
p = 1.0
G0 = ct.TransferFunction([1.0], [1.0, -p])   # 1 / (s - p)

# Proportional controller
K = 2.0
C = ct.TransferFunction([K], [1.0])

# Pure time delay (L seconds), approximated by first-order Pade
L_delay = 0.05  # 50 ms delay
num_d, den_d = ct.pade(L_delay, 1)
Delay = ct.TransferFunction(num_d, den_d)

# Open-loop with delay
L_sys = C * G0 * Delay

# Nyquist plot
plt.figure()
ct.nyquist_plot(L_sys)
plt.title("Nyquist plot: unstable plant with time delay")

# Gain and phase margins (approximate, due to Pade)
gm, pm, wg, wp = ct.margin(L_sys)
print(f"Gain margin: {gm:.3f}, phase margin: {pm:.2f} deg")
print(f"Gain crossover frequency: {wg:.3f} rad/s")

# Approximate delay margin using phase margin (in rad)
if wp is not None and pm > 0:
    pm_rad = pm * np.pi / 180.0
    L_max = pm_rad / wp
    print(f"Approximate delay margin L_max ≈ {L_max:.4f} s")
else:
    print("Phase margin not well defined (possibly unstable open loop).")

plt.show()
