import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Physical parameters for a DC motor / robotic joint approximation
J = 0.01   # inertia
b = 0.1    # viscous friction
Km = 1.0   # motor torque constant

# Transfer function G(s) = Km / (s (J s + b))
s = ct.TransferFunction.s
G = Km / (s * (J * s + b))

# 1) Root locus plot and gain selection
plt.figure()
rlocus_data = ct.root_locus(G, kvect=np.linspace(0, 200, 400), plot=True)
# rlocus_data is (poles, gains); can be post-processed numerically

# Example: pick a gain K that gives acceptable damping (inspect plot)
K_des = 50.0
L = K_des * G  # loop transfer function with proportional gain
T = ct.feedback(L, 1)  # closed-loop transfer function

# 2) Bode plot and stability margins
plt.figure()
mag, phase, omega = ct.bode(L, dB=True, Hz=False, omega_limits=(0.1, 100), omega_num=500)
gm, pm, wgm, wpm = ct.margin(L)
print("Gain margin (dB):", 20 * np.log10(gm) if gm is not None else None)
print("Phase margin (deg):", pm)
print("Gain crossover rad/s:", wgm)
print("Phase crossover rad/s:", wpm)

# 3) Nyquist plot
plt.figure()
ct.nyquist_plot(L)

plt.show()
