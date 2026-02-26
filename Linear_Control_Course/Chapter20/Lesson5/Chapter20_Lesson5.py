import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Plant: G(s) = K / ((T_m s + 1) s)
K = 50.0
T_m = 0.05
G = ctrl.TransferFunction([K], [T_m, 1.0, 0.0])

# Feedback-side PID gains (designed e.g. by root locus or frequency methods)
Kp = 20.0
Ki = 200.0
Kd = 0.02

# 2-DOF weights
b = 0.3   # proportional weight on reference
c = 0.0   # derivative weight on reference (often 0 for step commands)

# Cy(s) = Kp + Ki/s + Kd s
s = ctrl.TransferFunction([1, 0], [1])
Cy = Kp + Ki/s + Kd * s

# Cr(s) = Kp b + Ki/s + Kd c s
Cr = Kp * b + Ki/s + Kd * c * s

# Closed-loop transfer functions
T_r = ctrl.minreal(G * Cr / (1 + G * Cy))
T_d = ctrl.minreal(G / (1 + G * Cy))

# Simulate step in reference with no disturbance
t = np.linspace(0, 0.5, 2000)
t_out, y_r = ctrl.step_response(T_r, T=t)

# Simulate disturbance step (negative sign: load torque opposing motion)
Td = -T_d
t_out2, y_d = ctrl.step_response(Td, T=t)

plt.figure()
plt.plot(t_out, y_r, label="response to reference step")
plt.plot(t_out2, y_d, label="response to disturbance step")
plt.xlabel("time [s]")
plt.ylabel("joint angle [rad]")
plt.grid(True)
plt.legend()
plt.show()
