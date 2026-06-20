import numpy as np
import control as ctrl   # python-control library
# In robotics, see also: roboticstoolbox-python for multi-joint models.

# Plant P1(s) = 1 / (s (s + 1))
s = ctrl.tf([1, 0], [1])        # Laplace variable s
P1 = 1 / (s * (s + 1))

# Controller parameters (example numbers)
Kc  = 12.0
zi  = 0.8    # integral zero
zl  = 6.0    # lead zero
pl  = 1.5    # lead pole

Ci = (s + zi) / s               # PI part
Cl = (s + zl) / (s + pl)        # lead part
C1 = Kc * Ci * Cl

L1 = C1 * P1                    # open loop
T1 = ctrl.feedback(L1, 1)       # closed loop from r to y

# Bode plot and margins
omega = np.logspace(-2, 2, 400)
mag, phase, wout = ctrl.bode(L1, omega, Plot=False)
gm, pm, wgc, wpc = ctrl.margin(L1)
print("Gain margin:", gm, "Phase margin:", pm, "deg")

# Step response
t, y = ctrl.step_response(T1)
import matplotlib.pyplot as plt
plt.figure()
plt.plot(t, y)
plt.xlabel("t [s]")
plt.ylabel("position")
plt.title("Servo step response with loop-shaped PI+lead")
plt.grid(True)
plt.show()
