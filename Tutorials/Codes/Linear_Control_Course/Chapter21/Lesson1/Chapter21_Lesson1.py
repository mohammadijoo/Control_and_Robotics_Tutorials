import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

# DC motor parameters (simple model)
J = 0.01   # inertia
b = 0.1    # viscous friction
K = 1.0    # effective torque gain

# Plant G(s) = K / (J s^2 + b s)
numG = [K]
denG = [J, b, 0.0]
G = ctrl.TransferFunction(numG, denG)

# PI controller C(s) = Kp + Ki / s
Kp = 20.0
Ki = 40.0
C = ctrl.TransferFunction([Kp, Ki], [1.0, 0.0])

L = C * G          # loop transfer
T = ctrl.feedback(L, 1)  # closed loop from r to y

# Bode plot of L(jw)
mag, phase, omega = ctrl.bode(L, dB=True, Hz=False, omega_limits=(1e-1, 1e3), omega_num=500)

# Step response of closed loop
t, y = ctrl.step_response(T)
plt.figure()
plt.plot(t, y)
plt.xlabel("time [s]")
plt.ylabel("position response")
plt.grid(True)

plt.show()

# In a robotics context, such a loop would be embedded in a joint controller.
# Libraries such as 'roboticstoolbox' for Python can provide the multi-DOF
# manipulator model, while 'python-control' handles single-joint loop analysis.
