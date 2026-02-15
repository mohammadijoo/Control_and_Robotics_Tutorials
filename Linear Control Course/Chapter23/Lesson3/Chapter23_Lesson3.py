import numpy as np
import control as ct

# Nominal robot joint parameters (simplified)
J = 0.01   # kg m^2
b = 0.1    # N m s/rad
Km = 1.0   # gain from voltage to torque*gear ratio units

# Flexible mode parameters (unmodeled in P0)
zeta_f = 0.05
w_f = 200.0  # rad/s, high-frequency flexible mode

# Transfer function variable
s = ct.TransferFunction.s

# Nominal rigid-body plant P0(s) = Km / (J s^2 + b s)
P0 = Km / (J * s**2 + b * s)

# Flexible mode factor Gf(s) = w_f^2 / (s^2 + 2 zeta_f w_f s + w_f^2)
Gf = (w_f**2) / (s**2 + 2 * zeta_f * w_f * s + w_f**2)

# True plant including flexible mode (unknown in nominal design)
P_true = P0 * Gf

# Multiplicative deviation Delta_m(s) = P_true(s)/P0(s) - 1 = Gf(s) - 1
Delta_m = Gf - 1

# Simple proportional controller (for illustration)
Kp = 5.0
C = Kp

L0 = C * P0                 # nominal loop
T0 = ct.feedback(L0, 1)     # nominal complementary sensitivity

# Candidate multiplicative weight Wm(s)
alpha = 1.0 / (2.0 * zeta_f)
Wm = alpha * (s / w_f) / (1 + s / w_f)

# Frequency grid
w = np.logspace(0, 4, 500)  # 1 .. 10000 rad/s

# Frequency responses
_, mag_T0, _ = ct.freqresp(T0, w)
_, mag_Wm, _ = ct.freqresp(Wm, w)
_, mag_Dm, _ = ct.freqresp(Delta_m, w)

# Evaluate robust stability margin sup |Wm(jw) T0(jw)|
prod_mag = (mag_Wm * mag_T0).ravel()
max_prod = np.max(prod_mag)

print("sup_w |Wm(jw) T0(jw)| =", float(max_prod))

# Check that the actual multiplicative error is covered by the weight:
# |Delta_m(jw)| <= |Wm(jw)| ? (not guaranteed, but indicative)
ratio = (mag_Dm / (mag_Wm + 1e-8)).ravel()
print("max_w |Delta_m(jw)| / |Wm(jw)| =", float(np.max(ratio)))

# NOTE (robotics context):
# In a more detailed model, parameters J and b could be obtained from a
# robotics toolbox, e.g.:
#   from roboticstoolbox import models
#   robot = models.DH.Puma560()
#   J = robot.inertia(q)[joint_index, joint_index]
# allowing uncertainty analysis directly on realistic joint dynamics.
