import numpy as np
import matplotlib.pyplot as plt

# Optional: Python Control Systems Library (install via: pip install control)
import control as ctl

# Plant parameters (e.g., simplified DC motor velocity loop)
K = 1.0      # DC gain
tau = 0.5    # time constant [s]

# Desired closed-loop specs (approximate)
zeta = 0.7
omega_n = 3.0

# PI gains from Section 5 formulas
Kp = (2.0 * zeta * omega_n * tau - 1.0) / K
Ki = (tau * omega_n**2) / K

print("Designed gains: Kp =", Kp, ", Ki =", Ki)

# Continuous-time transfer functions
s = ctl.TransferFunction.s
G_p = K / (tau * s + 1)
G_c = Kp + Ki / s    # PI controller

L = G_c * G_p        # open loop
T = ctl.feedback(L, 1)  # closed loop (unity feedback)

# Step response
t = np.linspace(0, 5, 1000)
t, y = ctl.step_response(T, T=t)

e = 1.0 - y          # error to unit step
e_ss = e[-1]
print("Approx steady-state error (step):", e_ss)

plt.figure()
plt.plot(t, y, label="y(t)")
plt.plot(t, np.ones_like(t), "--", label="reference")
plt.xlabel("Time [s]")
plt.ylabel("Output")
plt.title("PI-controlled first-order plant")
plt.legend()
plt.grid(True)
plt.show()
