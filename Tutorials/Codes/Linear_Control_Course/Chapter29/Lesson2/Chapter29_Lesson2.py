import numpy as np
import matplotlib.pyplot as plt

# Optional: python-control library (pip install control)
import control as ctl

# Physical parameters (example values)
J_eq = 0.01     # [kg*m^2]
b_eq = 0.001    # [N*m*s/rad]
K_t = 0.1       # [N*m/A]
K_c = 2.0       # [A/unit command]
N   = 50.0      # gear ratio

Kv = (K_t * K_c) / (N * b_eq)
tau_m = J_eq / b_eq

print("Kv =", Kv, "tau_m =", tau_m)

# Plant Gp(s) = Kv / (s * (tau_m*s + 1))
s = ctl.TransferFunction.s
Gp = Kv / (s * (tau_m * s + 1))

# Design Kp from desired zeta (using P-only design relation)
zeta = 0.7
Kp = 1.0 / (4.0 * (zeta**2) * Kv * tau_m)

C = ctl.TransferFunction([Kp], [1.0])
L = C * Gp
T = ctl.feedback(L, 1.0)  # unity feedback

t = np.linspace(0, 0.5, 1000)
t, y = ctl.step_response(T, t)

plt.figure()
plt.plot(t, y)
plt.xlabel("Time [s]")
plt.ylabel("Joint position theta(t) [rad]")
plt.title("Position servo step response (P control)")
plt.grid(True)
plt.show()
