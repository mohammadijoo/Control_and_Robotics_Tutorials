import numpy as np
import control as ctl  # python-control library
# (pip install control)

# Plant: G(s) = 1 / (s (s + 2))
G = ctl.TransferFunction([1.0], [1.0, 2.0, 0.0])

# Desired dominant pole parameters
zeta = 0.5
wn = 4.0
sd = -zeta * wn + 1j * wn * np.sqrt(1.0 - zeta**2)  # -2 + j*3.464...

# Compensator zero chosen analytically
zc = 8.0
Cz = ctl.TransferFunction([1.0, zc], [1.0])  # C(s) = s + 8

# Open-loop with compensator (without the gain K yet)
L_base = Cz * G

# Use magnitude condition to compute gain K so that sd is on the locus
# For L(s) = K * L_base(s), we need |K * L_base(sd)| = 1
# so K = 1 / |L_base(sd)|
num_eval = np.polyval(L_base.num[0][0], sd)
den_eval = np.polyval(L_base.den[0][0], sd)
L_val = num_eval / den_eval
K = 1.0 / abs(L_val)

print(f"Designed gain K ≈ {K:.3f}")

# Closed-loop transfer function with unity feedback
C = K * Cz
T = ctl.feedback(C * G, 1)

# Inspect dominant poles, step response, and classical performance metrics
print("Closed-loop poles:", ctl.pole(T))

t, y = ctl.step_response(T)
info = ctl.step_info(T)
print("Step info:", info)

# In a robotics context, G(s) could be obtained from a joint model, e.g.,
# using the Robotics Toolbox for Python, then linearized and passed into
# the same design workflow.
