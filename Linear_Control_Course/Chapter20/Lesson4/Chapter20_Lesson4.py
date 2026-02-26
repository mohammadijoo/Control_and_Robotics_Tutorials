import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Plant: approximate robot joint position dynamics
# G(s) = 1 / (s (s + 1))
s = ct.TransferFunction.s
G = 1 / (s * (s + 1))

# Feedback PID controller C(s)
Kp = 30.0
Ki = 70.0
Kd = 2.0
C = Kp + Ki / s + Kd * s

# Loop transfer function
L = C * G

# 1-DOF closed-loop (no prefilter)
Tyr_1dof = ct.minreal(L / (1 + L))
Tyd_1dof = ct.minreal(G / (1 + L))  # disturbance at plant input

# 2-DOF with first-order prefilter Q(s) = 1 / (Tq s + 1)
Tq = 0.2  # prefilter time constant
Q = 1 / (Tq * s + 1)

Tyr_2dof = ct.minreal(G * Q / (1 + L))
Tyd_2dof = ct.minreal(G / (1 + L))  # same as 1-DOF

# Step response from reference
t = np.linspace(0.0, 5.0, 500)
t1, y1 = ct.step_response(Tyr_1dof, T=t)
t2, y2 = ct.step_response(Tyr_2dof, T=t)

plt.figure()
plt.plot(t1, y1, label="1-DOF tracking")
plt.plot(t2, y2, linestyle="--", label="2-DOF tracking with Q(s)")
plt.xlabel("time [s]")
plt.ylabel("joint position")
plt.legend()
plt.grid(True)

# Disturbance response: simulate step load torque at plant input
# Use superposition with D(s) entering through Tyd(s)
t3, yd1 = ct.step_response(Tyd_1dof, T=t)
t4, yd2 = ct.step_response(Tyd_2dof, T=t)

plt.figure()
plt.plot(t3, yd1, label="1-DOF disturbance response")
plt.plot(t4, yd2, linestyle="--", label="2-DOF disturbance response (same)")
plt.xlabel("time [s]")
plt.ylabel("joint position deviation")
plt.legend()
plt.grid(True)

plt.show()
