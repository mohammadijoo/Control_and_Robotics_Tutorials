import numpy as np
import control  # python-control library, often used in robotics courses
# (Optional) from roboticstoolbox import DHRobot, RevoluteDH  # robot models

# Plant: P(s) = 1 / (s (s + 1) (s + 2))
P = control.tf([1.0], [1.0, 3.0, 2.0, 0.0])

# Two PI controllers:
# "Conservative" (lower bandwidth, larger phase margin)
Kp1, Ki1 = 2.0, 1.0
C1 = control.tf([Kp1, Ki1], [1.0, 0.0])

# "Aggressive" (higher bandwidth, smaller phase margin)
Kp2, Ki2 = 6.0, 4.0
C2 = control.tf([Kp2, Ki2], [1.0, 0.0])

L1 = C1 * P
L2 = C2 * P

T1 = control.feedback(L1, 1)  # closed-loop from r to y
T2 = control.feedback(L2, 1)

# Bode plots to compare loop shapes
w = np.logspace(-2, 2, 500)
mag1, phase1, wout1 = control.bode(L1, w, Plot=False)
mag2, phase2, wout2 = control.bode(L2, w, Plot=False)

bw1 = control.bandwidth(T1)
bw2 = control.bandwidth(T2)

print("Conservative controller:  bandwidth ~", bw1, "rad/s")
print("Aggressive controller:    bandwidth ~", bw2, "rad/s")

# Step responses to compare time-domain performance
t = np.linspace(0, 10, 1000)
t1, y1 = control.step_response(T1, T=t)
t2, y2 = control.step_response(T2, T=t)

# Compute overshoot
Mp1 = (np.max(y1) - 1.0) * 100.0
Mp2 = (np.max(y2) - 1.0) * 100.0
print("Conservative overshoot ~ {:.1f}%".format(Mp1))
print("Aggressive overshoot   ~ {:.1f}%".format(Mp2))

# Optional: control effort for a unit step
U1 = control.forced_response(C1 / (1 + L1), T=t, U=np.ones_like(t))[1]
U2 = control.forced_response(C2 / (1 + L2), T=t, U=np.ones_like(t))[1]
print("Peak control effort (conservative):", np.max(np.abs(U1)))
print("Peak control effort (aggressive):  ", np.max(np.abs(U2)))
