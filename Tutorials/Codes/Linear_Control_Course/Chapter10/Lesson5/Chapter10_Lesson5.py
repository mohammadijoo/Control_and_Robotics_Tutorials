import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Plant: G(s) = 100 / (s (0.1 s + 1))
s = ctrl.TransferFunction.s
G = 100 / (s * (0.1 * s + 1))

# --- Case Study 1: proportional control only ---
Kc1 = 0.05
C1 = Kc1
L1 = C1 * G              # open loop
T1 = ctrl.feedback(L1, 1)  # closed loop with unity feedback

poles_T1 = ctrl.pole(T1)
print("Case 1 poles:", poles_T1)

# --- Case Study 2: lead compensator ---
Kc2 = 0.2
C_lead = Kc2 * (s + 10) / (s + 20)
L2 = C_lead * G
T2 = ctrl.feedback(L2, 1)
poles_T2 = ctrl.pole(T2)
print("Case 2 poles:", poles_T2)

# Root locus for compensated loop
plt.figure()
ctrl.rlocus(L2, kvect=np.linspace(0, 1.0, 200))
plt.title("Root locus of lead-compensated loop")

# Step responses for comparison
t = np.linspace(0, 2.0, 1000)
t1, y1 = ctrl.step_response(T1, t)
t2, y2 = ctrl.step_response(T2, t)

plt.figure()
plt.plot(t1, y1, label="Case 1: Kc = 0.05")
plt.plot(t2, y2, label="Case 2: lead C_l(s)")
plt.xlabel("Time [s]")
plt.ylabel("Joint angle response")
plt.legend()
plt.grid(True)
plt.show()

# Robotics context (conceptual sketch):
# from roboticstoolbox import DHRobot, RevoluteDH
# Define a 1-DOF robot joint whose dynamics are approximated by G(s);
# the closed-loop transfer T2(s) can be used within a higher-level
# trajectory generator or simulation loop.
