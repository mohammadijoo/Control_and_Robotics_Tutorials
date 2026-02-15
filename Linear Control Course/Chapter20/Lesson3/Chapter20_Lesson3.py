import numpy as np
import matplotlib.pyplot as plt

# Robotics-oriented control toolbox
import control as ct  # python-control

# Closed-loop model (without pre-filter): underdamped 2nd-order
wn = 10.0   # natural frequency [rad/s]
zeta = 0.2  # damping ratio (underdamped)

num_T0 = [wn**2]
den_T0 = [1.0, 2.0*zeta*wn, wn**2]
T0 = ct.tf(num_T0, den_T0)

# Desired critically damped model with same wn
num_M = [wn**2]
den_M = [1.0, 2.0*wn, wn**2]
M = ct.tf(num_M, den_M)

# Model-matching pre-filter F(s) = M(s) / T0(s)
F_match = ct.minreal(M / T0, verbose=False)

# Alternative: simple first-order low-pass pre-filter
tau_f = 0.1  # design time constant
F_lp = ct.tf([1.0], [tau_f, 1.0])

# Combined tracking transfer functions
T_match = ct.minreal(F_match * T0, verbose=False)
T_lp    = ct.minreal(F_lp * T0, verbose=False)

# Time vector for simulation
t = np.linspace(0.0, 2.0, 500)

# Step responses
t0, y0 = ct.step_response(T0, T=t)        # no pre-filter
t1, y1 = ct.step_response(T_match, T=t)   # model-matching F(s)
t2, y2 = ct.step_response(T_lp, T=t)      # low-pass F(s)

plt.figure()
plt.plot(t0, y0, label="T0(s) (no pre-filter)")
plt.plot(t1, y1, label="F_match * T0(s)")
plt.plot(t2, y2, label="F_lp * T0(s)")
plt.xlabel("Time [s]")
plt.ylabel("Output y(t)")
plt.title("Effect of Reference Pre-Filters on Step Tracking")
plt.grid(True)
plt.legend()

plt.show()
