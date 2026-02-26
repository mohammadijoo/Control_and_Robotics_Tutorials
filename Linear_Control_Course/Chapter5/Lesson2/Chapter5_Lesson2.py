import numpy as np
import matplotlib.pyplot as plt

# Parameters of first-order system: G(s) = K / (tau s + 1)
K = 2.0
tau = 0.5

# Time grid
t_end = 5.0
n_points = 2000
t = np.linspace(0.0, t_end, n_points)

# Analytical responses for unit inputs (U0 = 1, R = 1)
u_step = np.ones_like(t)          # unit step
u_ramp = t                        # unit ramp
# Approximate unit impulse as a narrow pulse of width dt and area 1
dt = t[1] - t[0]
u_imp = np.zeros_like(t)
u_imp[0] = 1.0 / dt

# Closed-form responses
y_step = K * (1.0 - np.exp(-t / tau))           # y_step(t)
y_ramp = K * (t - tau * (1.0 - np.exp(-t / tau)))  # y_ramp(t)
y_imp = (K / tau) * np.exp(-t / tau)            # y_imp(t)

# (Optional) numerical integration of the ODE tau dy/dt + y = K u(t)
def simulate_first_order(u):
    y = np.zeros_like(t)
    for k in range(1, len(t)):
        dydt = (-y[k-1] + K * u[k-1]) / tau
        y[k] = y[k-1] + dt * dydt
    return y

y_step_num = simulate_first_order(u_step)
y_ramp_num = simulate_first_order(u_ramp)
y_imp_num = simulate_first_order(u_imp)

# Plotting
plt.figure()
plt.plot(t, y_step, label="analytic step")
plt.plot(t, y_step_num, "--", label="numerical step")
plt.xlabel("t [s]")
plt.ylabel("y_step(t)")
plt.legend()

plt.figure()
plt.plot(t, y_ramp, label="analytic ramp")
plt.plot(t, y_ramp_num, "--", label="numerical ramp")
plt.xlabel("t [s]")
plt.ylabel("y_ramp(t)")
plt.legend()

plt.figure()
plt.plot(t, y_imp, label="analytic impulse")
plt.plot(t, y_imp_num, "--", label="numerical impulse")
plt.xlabel("t [s]")
plt.ylabel("y_imp(t)")
plt.legend()

plt.show()

# In a robotics context:
# - G(s) may model a motor velocity loop.
# - Libraries such as 'control' (python-control) can define tf(K, [tau, 1])
#   and compute step_response/impulse_response for joint-level design.
