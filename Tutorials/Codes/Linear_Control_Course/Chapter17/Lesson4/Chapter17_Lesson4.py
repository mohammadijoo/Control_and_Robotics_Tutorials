import numpy as np
import matplotlib.pyplot as plt

# Core control library
import control  # pip install control

# (Optional) robotics toolbox for realistic robot models
# import roboticstoolbox as rtb

# Robot joint parameters (simplified)
J = 0.01   # inertia
B = 0.1    # viscous friction
K = 1.0    # gain from voltage to torque*gear etc.

Kp = 20.0  # proportional gain to be tuned

# Continuous-time transfer functions
s = control.TransferFunction.s
P = K / (J * s**2 + B * s)  # plant
C = Kp                     # P controller
L = C * P                  # open loop
T = control.feedback(L, 1) # closed loop (unity feedback)

# Compute stability margins
gm, pm, w_pc, w_gc = control.margin(L)
gm_db = 20 * np.log10(gm)

print(f"Gain margin (linear): {gm:.2f}")
print(f"Gain margin (dB):    {gm_db:.2f}")
print(f"Phase margin (deg):  {pm:.2f}")
print(f"Phase crossover w_pc (rad/s): {w_pc:.2f}")
print(f"Gain crossover w_gc  (rad/s): {w_gc:.2f}")

# Approximate delay margin from phase margin
pm_rad = pm * np.pi / 180.0
tau_d = pm_rad / w_gc
print(f"Approximate delay margin tau_d ~ {tau_d:.4f} s")

# Bode plot with margins overlaid
mag, phase, w = control.bode_plot(
    L,
    dB=True,
    Hz=False,
    omega_limits=(0.1, 1e3),
    margins=True,   # draws lines for GM/PM
    grid=True
)

# Closed-loop step response
t, y = control.step_response(T)
plt.figure()
plt.plot(t, y)
plt.xlabel("t [s]")
plt.ylabel("theta(t) [rad]")
plt.title("Closed-loop step response of robot joint (P control)")
plt.grid(True)

plt.show()
