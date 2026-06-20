import numpy as np
import control  # python-control, MATLAB-like control library

# Example: simple joint servo loop, L(s) = K/(s (s + 1))
K = 1.0
s = control.TransferFunction.s
P = 1 / (s * (s + 1))
C = K
L = C * P

# Compute gain margin, phase margin, and crossover frequencies
Gm, Pm, Wcg, Wcp = control.margin(L)

print("Gain margin (abs):", Gm)
print("Gain margin (dB) :", 20 * np.log10(Gm) if Gm not in (0, np.inf) else Gm)
print("Phase margin (deg):", Pm)
print("Gain crossover frequency (rad/s):", Wcg)
print("Phase crossover frequency (rad/s):", Wcp)

# Bode plot with margins highlighted
import matplotlib.pyplot as plt
control.margin(L)
plt.show()

# Robotics context: using a joint inertial model from a robotics toolbox
try:
    from roboticstoolbox import DHRobot, RevoluteDH

    # Very simple 1-DOF link (not a realistic full model)
    link = RevoluteDH(a=0.0, d=0.0, alpha=0.0)
    robot = DHRobot([link], name="single_joint")

    # Suppose we linearize the joint dynamics around some configuration and speed
    # and obtain an equivalent first/second-order plant P_joint(s).
    # Here we just re-use P as a placeholder.
    P_joint = P
    L_joint = C * P_joint
    Gm_j, Pm_j, Wcg_j, Wcp_j = control.margin(L_joint)
    print("Joint servo phase margin (deg):", Pm_j)
except ImportError:
    print("roboticstoolbox not installed; skipping robotics-specific example.")
