import numpy as np
import control  # python-control: classical control for robotics and mechatronics

# Nominal DC motor velocity plant: G(s) = K / (J s + B)
J_nom = 0.01   # nominal inertia
B_nom = 0.1    # viscous friction
K_gain = 1.0   # motor gain

s = control.TransferFunction([1, 0], [1])
G_nom = K_gain / (J_nom * s + B_nom)

# Proportional velocity controller (this could be part of a cascaded robot joint loop)
Kc = 2.0
C = Kc

L_nom = C * G_nom
T_nom = control.feedback(L_nom, 1)

# Nominal margins
gm, pm, wcg, wcp = control.margin(L_nom)
print("Nominal gain margin (dB):", 20 * np.log10(gm))
print("Nominal phase margin (deg):", pm)
print("Gain crossover (rad/s):", wcp)

# Now sweep inertia to represent payload changes (e.g., ±50 %)
J_values = np.linspace(0.5 * J_nom, 1.5 * J_nom, 7)

def closed_loop_for_inertia(J):
    G = K_gain / (J * s + B_nom)
    L = C * G
    T = control.feedback(L, 1)
    return L, T

for J in J_values:
    L, T = closed_loop_for_inertia(J)
    gm_J, pm_J, wcg_J, wcp_J = control.margin(L)
    cl_poles = control.pole(T)
    stable = np.all(np.real(cl_poles) < 0)
    print("\nJ = {:.4f}".format(J))
    print("  phase margin (deg): {:.2f}".format(pm_J))
    print("  stable?:", stable)
    print("  closed-loop poles:", cl_poles)

# Observe that phase margin does not always give a clear picture of robustness
# when J deviates significantly from its nominal value.
