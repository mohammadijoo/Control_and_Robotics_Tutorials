import numpy as np
import matplotlib.pyplot as plt

# If python-control is installed:
# pip install control
import control as ctrl

# Physical parameters (e.g., robot joint approximated as mass-spring-damper)
M = 1.0   # kg
B = 0.5   # N s/m
K = 4.0   # N/m

# PD gains
Kp = 16.0
Kd = 4.0

# Plant: M d2y/dt2 + B dy/dt + K y = u
# Transfer function from u to y: 1 / (M s^2 + B s + K)
s = ctrl.TransferFunction.s
G = 1.0 / (M * s**2 + B * s + K)

# PD controller: C(s) = Kp + Kd s
C = Kd * s + Kp

# Unity feedback closed-loop from r to y
T = ctrl.feedback(C * G, 1)

# Step response (unit step reference)
t = np.linspace(0, 10, 1000)
t, y = ctrl.step_response(T, T=t)

# Compute effective zeta and omega_n from formulas
omega_n = np.sqrt((K + Kp) / M)
zeta = (B + Kd) / (2.0 * np.sqrt(M * (K + Kp)))

print("Effective omega_n =", omega_n)
print("Effective zeta    =", zeta)

plt.figure()
plt.plot(t, y, label="y(t)")
plt.axhline(1.0, linestyle="--", label="reference")
plt.xlabel("Time [s]")
plt.ylabel("Position")
plt.title("PD-Controlled Mass-Spring-Damper Step Response")
plt.grid(True)
plt.legend()
plt.show()
