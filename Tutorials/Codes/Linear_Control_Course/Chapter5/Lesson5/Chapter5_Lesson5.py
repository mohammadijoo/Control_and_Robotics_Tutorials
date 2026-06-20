import numpy as np
import matplotlib.pyplot as plt
import control as ct  # python-control library

# Exact second-order plant: G(s) = 1 / ((1 + s)(1 + 0.1 s))
G_full = ct.tf([1.0], np.polymul([1.0, 1.0], [0.1, 1.0]))

# First-order approximation using dominant pole tau1 = 1 s
G_1st = ct.tf([1.0], [1.0, 1.0])

t = np.linspace(0, 8, 400)
t1, y_full = ct.step_response(G_full, T=t)
t2, y_1st = ct.step_response(G_1st, T=t)

plt.figure()
plt.plot(t1, y_full, label="Full model")
plt.plot(t2, y_1st, linestyle="--", label="1st-order approx")
plt.xlabel("Time [s]")
plt.ylabel("Output")
plt.legend()
plt.grid(True)
plt.show()
