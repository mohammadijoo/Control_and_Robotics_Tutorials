import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Laplace variable
s = ct.TransferFunction.s

# Inner and outer plants
G1 = 5 / (0.1 * s + 1)   # fast inner dynamics
G2 = 2 / (0.5 * s + 1)   # slower outer dynamics

# Inner and outer proportional controllers
k1 = 8.0   # inner-loop gain
k2 = 1.5   # outer-loop gain
C1 = k1
C2 = k2

# Inner closed loop: G_cl1(s) = C1*G1 / (1 + C1*G1)
G_cl1 = ct.feedback(C1 * G1, 1)

# Equivalent plant for outer loop
Geq = G_cl1 * G2

# Overall cascade closed loop
T_cascade = ct.feedback(C2 * Geq, 1)

# Compare with single-loop design (for reference)
G_single = G1 * G2
C_single = k1 * k2
T_single = ct.feedback(C_single * G_single, 1)

# Step responses
t = np.linspace(0, 5, 500)
t1, y_cas = ct.step_response(T_cascade, T=t)
t2, y_single = ct.step_response(T_single, T=t)

plt.figure()
plt.plot(t1, y_cas, label="Cascade closed loop")
plt.plot(t2, y_single, linestyle="--", label="Single-loop closed loop")
plt.xlabel("Time (s)")
plt.ylabel("Output y(t)")
plt.legend()
plt.grid(True)
plt.show()
