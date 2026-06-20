import numpy as np
import control as ctl  # python-control library
# optional: from roboticstoolbox import DHRobot  # robotics toolbox (not used directly here)

# Nominal parameters for a first-order joint actuator
K0 = 10.0     # nominal gain [rad/(V*s)]
tau0 = 0.05   # nominal time constant [s]

alpha = 0.2   # 20% gain uncertainty
beta = 0.1    # 10% time-constant uncertainty

# Nominal transfer function G0(s) = K0 / (tau0 s + 1)
s = ctl.tf([1, 0], [1])  # Laplace variable
G0 = K0 / (tau0 * s + 1)

# Frequency grid
w = np.logspace(0, 3, 300)   # 1 rad/s to 1000 rad/s

# Monte Carlo sampling of uncertain plants
n_samples = 200
DeltaM_samples = []

for k in range(n_samples):
    K = K0 * (1 + alpha * (2*np.random.rand() - 1))    # uniform in [1-alpha, 1+alpha]
    tau = tau0 * (1 + beta * (2*np.random.rand() - 1)) # uniform in [1-beta, 1+beta]
    G = K / (tau * s + 1)
    # frequency responses
    _, G0jw = ctl.freqresp(G0, w)
    _, Gjw = ctl.freqresp(G, w)
    DeltaM = (Gjw - G0jw) / G0jw   # multiplicative error
    DeltaM_samples.append(DeltaM.squeeze())

DeltaM_samples = np.array(DeltaM_samples)  # shape: (n_samples, n_freq)

# Empirical bound over samples
DeltaM_mag = np.abs(DeltaM_samples)
DeltaM_bound = DeltaM_mag.max(axis=0)   # max over samples at each frequency

# Design a simple bound: here we just exaggerate by 20% for safety
DeltaM_bound_safe = 1.2 * DeltaM_bound

# (Plotting code would use matplotlib; omitted here for brevity)
# In practice, plot DeltaM_bound_safe vs frequency and hand-fit a simple W_M(s).

# Example: define a crude first-order weight W_M(s) that roughly matches the bound
# W_M(s) = k_w * s/(s + w_c) + epsilon
k_w = 0.3
w_c = 1.0 / tau0
epsilon = 0.05
WM = k_w * s / (s + w_c) + epsilon

print("Nominal plant G0(s):", G0)
print("Example multiplicative weight W_M(s):", WM)
