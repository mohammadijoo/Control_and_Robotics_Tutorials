import numpy as np
import control  # python-control
# Optional: robotics toolbox for more complex robot models
# from roboticstoolbox import DHRobot, RevoluteDH

# Physical parameters
J = 0.01   # inertia
b = 0.1    # viscous friction

# Open-loop transfer function G(s) = 1 / (J s^2 + b s)
num = [1.0]
den = [J, b, 0.0]
G = control.TransferFunction(num, den)

print("Open-loop transfer function G(s):")
print(G)

poles = control.pole(G)
print("Poles:", poles)

def classify_poles(p):
    if np.any(np.real(p) > 0.0):
        return "unstable"
    if np.any(np.isclose(np.real(p), 0.0)):
        return "marginally stable (not asymptotically stable)"
    return "asymptotically stable"

print("Stability classification:", classify_poles(poles))

# Example: add simple proportional feedback on angle (position control)
Kp = 5.0
# Unity feedback: closed-loop from reference to angle
G_cl = control.feedback(Kp * G, 1)
poles_cl = control.pole(G_cl)
print("Closed-loop poles:", poles_cl)
print("Closed-loop stability:", classify_poles(poles_cl))

# For robotics, one can obtain linearized joint dynamics matrices (A,B)
# from more complex models and use:
# eigvals, _ = np.linalg.eig(A)
# print("Eigenvalues of A:", eigvals)
# print("State-space stability:", classify_poles(eigvals))
