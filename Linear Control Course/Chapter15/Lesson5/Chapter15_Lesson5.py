import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Third-order plant: G(s) = 1 / (s (s + 1) (s + 3))
# Denominator: s^3 + 4 s^2 + 3 s
num = [1.0]
den = [1.0, 4.0, 3.0, 0.0]
G = ctrl.TransferFunction(num, den)

# Sampling of candidate gains
gains = [3.0, 9.0, 12.0]
fig, ax = plt.subplots()

for K in gains:
    L = K * G
    # Nyquist plot (python-control draws both positive and negative frequencies)
    ctrl.nyquist_plot(L, omega_limits=(1e-2, 1e2), omega_num=400, ax=ax)
    # Closed-loop transfer function T(s) = L(s) / (1 + L(s))
    T = ctrl.feedback(L, 1)
    poles = ctrl.pole(T)
    print(f"K = {K}")
    print("Closed-loop poles:", poles)
    if np.all(np.real(poles) < 0.0):
        print("  Stable closed loop\n")
    else:
        print("  Unstable closed loop\n")

# Mark the critical point -1 + j 0
ax.plot([-1.0], [0.0], "x")
ax.set_title("Nyquist plots for different gains K")
ax.set_xlabel("Real axis")
ax.set_ylabel("Imag axis")
ax.grid(True)
plt.show()

# In a robotics workflow, the chosen K would be used as the proportional gain
# of a joint controller (e.g., in a ros_control PID loop).
