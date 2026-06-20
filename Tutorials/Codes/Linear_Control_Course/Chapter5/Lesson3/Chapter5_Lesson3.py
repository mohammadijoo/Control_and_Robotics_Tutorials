import numpy as np
import matplotlib.pyplot as plt

# First-order parameters
K = 1.0          # static gain
tau = 0.4        # time constant (seconds)

# Time grid up to 5 tau
t_end = 5.0 * tau
t = np.linspace(0.0, t_end, 400)

# Analytic step response y(t) = K * (1 - exp(-t/tau))
y = K * (1.0 - np.exp(-t / tau))

plt.figure()
plt.plot(t, y, label="step response")
plt.axhline(K, linestyle="--", label="steady state K")
plt.axvline(tau, linestyle=":", label="time constant tau")
plt.xlabel("t [s]")
plt.ylabel("y(t)")
plt.title("First-order step response")
plt.legend()
plt.grid(True)
plt.show()

# Using python-control for the same system:
import control
G = control.tf([K], [tau, 1.0])
t2, y2 = control.step_response(G, T=t)
