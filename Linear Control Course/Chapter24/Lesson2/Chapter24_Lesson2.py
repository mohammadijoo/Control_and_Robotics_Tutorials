import numpy as np
import control  # python-control library, often used in robotics & mechatronics

# Plant P(s) = 1 / (s^2 + 2*zeta*wn*s + wn^2)
zeta = 0.7
wn   = 10.0
s = control.TransferFunction([1, 0], [0, 1])  # s variable
P = 1 / (s**2 + 2*zeta*wn*s + wn**2)

# Lead-lag type controller C(s) = K * (s + z) / s
K = 20.0
z = 2.0
C = K * (s + z) / s

# Loop L(s), sensitivity S(s), complementary sensitivity T(s)
L = C * P
S = 1 / (1 + L)
T = L / (1 + L)

# Performance and uncertainty weighting functions
M_P, A_P, wP = 1.5, 1e-3, 1.0   # low-freq perf weight
M_N, A_N, wN = 2.0, 1e-3, 50.0  # high-freq noise weight
alpha, wD    = 1.5, 30.0        # uncertainty weight

W_P = (s/M_P + wP) / (s + wP*A_P)
W_N = (s/M_N + wN) / (s + wN*A_N)
W_D = (s/alpha + wD) / (s + wD)

# Frequency grid (rad/s)
w = np.logspace(-2, 3, 500)

# Frequency responses
_, S_mag, _  = control.bode(S,  w, Plot=False)
_, T_mag, _  = control.bode(T,  w, Plot=False)
_, WP_mag, _ = control.bode(W_P, w, Plot=False)
_, WN_mag, _ = control.bode(W_N, w, Plot=False)
_, WD_mag, _ = control.bode(W_D, w, Plot=False)

# Pointwise robustness & performance checks
perf_metric   = WP_mag * S_mag          # |W_P S|
noise_metric  = WN_mag * T_mag          # |W_N T|
robust_metric = WD_mag * T_mag          # |W_D T|
combined      = perf_metric + robust_metric

print("max |W_P(jw) S(jw)|      =", np.max(perf_metric))
print("max |W_N(jw) T(jw)|      =", np.max(noise_metric))
print("max |W_D(jw) T(jw)|      =", np.max(robust_metric))
print("max (|W_P S| + |W_D T|)  =", np.max(combined))

# In a robotics context, P(s) could be a linearized joint or axis model
# obtained from a more complex multi-DOF model using robotics toolboxes
# (e.g. roboticstoolbox-python) and then reduced to a SISO transfer function.
