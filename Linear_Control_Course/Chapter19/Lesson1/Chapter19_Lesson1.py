import numpy as np
import control as ct  # pip install control

# Plant: G(s) = 1 / (s (s + 2))
s = ct.TransferFunction.s
G = 1 / (s * (s + 2))

# Design parameters for the lead compensator
# Desired additional phase (deg) and safety
phi_req_deg = 30.0
phi_safety_deg = 5.0
phi_max_deg = phi_req_deg + phi_safety_deg

phi_max = np.deg2rad(phi_max_deg)

# Compute alpha from sin(phi_max) = (alpha - 1)/(alpha + 1)
alpha = (1 + np.sin(phi_max)) / (1 - np.sin(phi_max))

# Choose desired new crossover frequency (rad/s)
omega_c_star = 2.0  # for example

# Compute T, zero, and pole
T = 1.0 / (omega_c_star * np.sqrt(alpha))
z_lead = -1.0 / (alpha * T)
p_lead = -1.0 / T

print("alpha =", alpha)
print("Lead zero at s =", z_lead)
print("Lead pole at s =", p_lead)

# Construct lead compensator C(s) = Kc (1 + alpha T s) / (1 + T s)
Kc = 1.0
C_base = Kc * (1 + alpha * T * s) / (1 + T * s)

# Adjust Kc so that |L(j w_c_star)| ≈ 1
mag, phase, omega = ct.bode(C_base * G, [omega_c_star], Plot=False)
Kc = 1.0 / mag[0]
C = Kc * C_base

print("Adjusted Kc =", Kc)

# Closed-loop system with unity feedback
T_cl = ct.feedback(C * G, 1)

# Time response
t = np.linspace(0, 10, 1000)
t_out, y_out = ct.step_response(T_cl, t)

# Example: print some performance indicators
print("Final value:", y_out[-1])
print("Approx. rise time (10-90%):",
      t_out[np.where(y_out > 0.9)[0][0]] - t_out[np.where(y_out > 0.1)[0][0]])
