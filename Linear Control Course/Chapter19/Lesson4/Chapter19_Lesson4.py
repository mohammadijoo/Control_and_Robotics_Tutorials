import numpy as np
import control as ctl
# If needed: pip install control
# For robotics context: pip install roboticstoolbox-python

# --- Plant: 1-DOF joint approximation ---
J = 0.01   # kg m^2
B = 0.1    # N m s/rad
Km = 0.5   # N m/A (effective gain)

s = ctl.TransferFunction.s
G = Km / (J * s**2 + B * s)

# --- Lead design (numbers chosen to give extra phase near wc) ---
wc_des = 10.0   # desired crossover rad/s
alpha = 0.2     # z1/p1 = alpha < 1 for lead

z1 = wc_des / np.sqrt(alpha)
p1 = wc_des * np.sqrt(alpha)

G_lead = (s + z1) / (s + p1)

# Preliminary gain so that |Kc * G_lead(j wc) * G(j wc)| ≈ 1
# Compute magnitude at wc_des
mag_plant, phase_plant, w = ctl.bode(G, [wc_des], Plot=False)
mag_lead, phase_lead, _ = ctl.bode(G_lead, [wc_des], Plot=False)
Kc = 1.0 / (mag_plant[0] * mag_lead[0])

# --- Lag design to improve steady-state position error ---
# Suppose we want 10x improvement in low-frequency error constants
beta = 10.0
wz2 = wc_des / 10.0
wp2 = wz2 / beta

G_lag = (s + wz2) / (s + wp2)

# Full controller: lead-lag with gain Kc
Gc = Kc * G_lead * G_lag

# Open-loop and closed-loop
L = Gc * G
T = ctl.feedback(L, 1)  # unity feedback

# Bode plot of compensated open-loop
ctl.bode(L, dB=True)

# Step response of joint position
t = np.linspace(0, 2.0, 1000)
t, y = ctl.step_response(T, T=t)

import matplotlib.pyplot as plt
plt.figure()
plt.plot(t, y)
plt.xlabel("Time [s]")
plt.ylabel("Joint position (rad)")
plt.title("Closed-loop step response with lead-lag compensator")
plt.grid(True)
plt.show()
