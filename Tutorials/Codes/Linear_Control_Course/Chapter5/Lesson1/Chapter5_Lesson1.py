import numpy as np
import matplotlib.pyplot as plt

# Control systems library (python-control)
import control as ct

# First-order system: G(s) = K / (tau s + 1)
K = 2.0
tau = 0.5
G = ct.tf([K], [tau, 1.0])

print("Transfer function G(s) =")
print(G)

# Example: unit-step response
t = np.linspace(0.0, 5.0, 500)
t_out, y_out = ct.step_response(G, T=t)

plt.figure()
plt.plot(t_out, y_out)
plt.xlabel("t [s]")
plt.ylabel("y(t)")
plt.title("Unit-step response of first-order system")
plt.grid(True)
plt.show()

# In robotics, this G(s) can approximate, e.g., a simple current loop
# of a DC motor used in a robot joint or wheel drive.
