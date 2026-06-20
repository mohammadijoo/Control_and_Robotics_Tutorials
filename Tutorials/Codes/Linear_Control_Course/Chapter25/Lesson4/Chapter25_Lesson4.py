import numpy as np
import control as ctrl

# Optional robotics toolbox import (for real robot models)
# import roboticstoolbox as rtb

# Laplace variable not needed explicitly in python-control
# Plant and disturbance path
G = ctrl.tf([1.0], [0.5, 1.0])       # G(s) = 1 / (0.5 s + 1)
Gd = ctrl.tf([0.2], [0.2, 1.0])      # G_d(s) = 0.2 / (0.2 s + 1)

# PI feedback controller C(s) = kp + ki/s
kp, ki = 8.0, 4.0
C = ctrl.tf([kp, ki], [1.0, 0.0])

# Ideal disturbance feedforward (may be improper)
F_ideal = -Gd / G
print("F_ideal(s) =", F_ideal)

# Check properness and stabilize with a first-order roll-off if needed
if not ctrl.isproper(F_ideal):
    # Add extra pole for realizability: alpha sets roll-off frequency
    alpha = 20.0
    rolloff = ctrl.tf([1.0], [1.0/alpha, 1.0])  # 1 / (s/alpha + 1)
    F = ctrl.minreal(F_ideal * rolloff, verbose=False)
else:
    F = ctrl.minreal(F_ideal, verbose=False)

print("Implementable F(s) =", F)

# Closed-loop transfer from disturbance to output with feedforward
Tyw = (G*F + Gd) / (1 + G*C)
print("T_yw(s) =", ctrl.minreal(Tyw, verbose=False))

# Simulate step disturbance
T = np.linspace(0, 10, 1000)
t, y = ctrl.step_response(Tyw, T)
# Plotting omitted; in practice, compare with Tyw_no_ff = Gd/(1+G*C)
