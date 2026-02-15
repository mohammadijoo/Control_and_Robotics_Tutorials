import numpy as np
import control as ctl
# If available: from roboticstoolbox import DHRobot, RevoluteDH

# Example plant: G(s) = 1 / (s (s + 1))
s = ctl.TransferFunction.s
G = 1 / (s * (s + 1))

# PI controller: C(s) = Kp + Ki / s
Kp = 4.0
Ki = 3.0
C = Kp + Ki / s

L = C * G                 # open-loop transfer function
S = 1 / (1 + L)           # sensitivity
T = L / (1 + L)           # complementary sensitivity

# Frequency grid for analysis
w = np.logspace(-2, 2, 400)
magS, phaseS, _ = ctl.bode(S, w, Plot=False)
magT, phaseT, _ = ctl.bode(T, w, Plot=False)

Ms = np.max(magS)
Mt = np.max(magT)
print("M_S =", Ms, "M_T =", Mt)

# Simple tuning loop: increase Kp until M_S exceeds a limit
Ms_target = 1.8
for Kp_test in np.linspace(1.0, 10.0, 10):
    C_test = Kp_test + Ki / s
    L_test = C_test * G
    S_test = 1 / (1 + L_test)
    magS_test, _, _ = ctl.bode(S_test, w, Plot=False)
    Ms_test = np.max(magS_test)
    if Ms_test > Ms_target:
        print("Kp", Kp_test, "violates M_S > target with M_S =", Ms_test)
        break

# Robotics remark:
# A typical workflow is:
# 1. Use a robotics library to derive or identify a linear joint model G_joint(s).
# 2. Use python-control to form L(s) = C(s) G_joint(s).
# 3. Evaluate S and T as above and adjust controller gains or filters.
