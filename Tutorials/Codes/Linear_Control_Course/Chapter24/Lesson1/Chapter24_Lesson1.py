import numpy as np
import control as ctl
import matplotlib.pyplot as plt

# Robot joint parameters (nominal)
J = 0.01     # kg m^2
b = 0.1      # N m s/rad
K_t = 0.5    # N m/A

# PI controller gains (to be tuned)
Kp = 20.0
Ki = 10.0

# Plant and controller transfer functions
G = ctl.tf([K_t], [J, b, 0.0])          # G(s) = K_t / (J s^2 + b s)
C = ctl.tf([Kp, Ki], [1.0, 0.0])        # C(s) = (Kp s + Ki) / s

L = C * G                               # open-loop
T = ctl.feedback(L, 1)                  # closed-loop

# Compute classical stability margins
gm, pm, w_gc, w_pc = ctl.margin(L)
gm_db = 20.0 * np.log10(gm) if gm is not None and gm > 0 else np.inf

print(f"Gain margin: {gm:.3f} ({gm_db:.2f} dB) at w_pc = {w_pc:.3f} rad/s")
print(f"Phase margin: {pm:.3f} deg at w_gc = {w_gc:.3f} rad/s")

# Approximate delay margin (radians for pm, then convert)
pm_rad = np.deg2rad(pm)
tau_max = pm_rad / w_gc
print(f"Approximate additional delay margin tau_max = {tau_max:.4f} s")

# Approximate admissible multiplicative uncertainty near crossover
delta_mult = 2.0 * np.sin(0.5 * pm_rad)
print(f"Approximate multiplicative uncertainty at crossover: delta_max = {delta_mult:.3f}")

# Bode plot with margin annotations
mag, phase, omega = ctl.bode(L, dB=True, Hz=False, omega_limits=(1e-1, 1e3), Plot=False)

plt.figure()
plt.subplot(2, 1, 1)
plt.semilogx(omega, 20.0 * np.log10(mag))
plt.ylabel("Magnitude (dB)")
plt.grid(True, which="both")

plt.subplot(2, 1, 2)
plt.semilogx(omega, phase * 180.0 / np.pi)
plt.xlabel("Frequency (rad/s)")
plt.ylabel("Phase (deg)")
plt.grid(True, which="both")

plt.tight_layout()
plt.show()

# NOTE (robotics context):
# In a robotics workflow, G and C are often derived using a library like
# 'roboticstoolbox' to obtain a linearized joint model:
#
#   from roboticstoolbox import DHRobot
#   # define robot, linearize around a configuration, then extract G(s)
#
# The same margin analysis on L(s) is then used to certify robust stability of
# the joint controller before deployment in ROS/ros2 control loops.
