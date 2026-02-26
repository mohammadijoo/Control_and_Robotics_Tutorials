import numpy as np
import matplotlib.pyplot as plt

# Plant parameters (e.g., robot joint position dynamics)
a = 1.0   # intrinsic damping coefficient
b = 1.0   # control effectiveness
K = 4.0   # proportional gain
r0 = 1.0  # desired position (setpoint)

# Simulation settings
dt = 0.001   # time step [s]
T  = 3.0     # total simulation time [s]
N  = int(T / dt)

t = np.linspace(0.0, T, N + 1)
y = np.zeros(N + 1)  # plant output (position)
r = r0 * np.ones(N + 1)  # constant reference

for k in range(N):
    # Measured output (ideal sensor)
    y_m = y[k]
    # Error
    e = r[k] - y_m
    # Proportional controller
    u = K * e
    # Plant dynamics: y_dot = -a y + b u
    y_dot = -a * y[k] + b * u
    # Euler integration
    y[k + 1] = y[k] + dt * y_dot

plt.figure()
plt.plot(t, r, linestyle="--", label="reference r(t)")
plt.plot(t, y, label="output y(t)")
plt.xlabel("time [s]")
plt.ylabel("signal")
plt.legend()
plt.title("First-order plant with proportional feedback")
plt.show()

# Note:
# In robotics applications, you might replace this simple model
# with a more detailed joint model and use robotics libraries like:
#   - python-control (for LTI analysis)
#   - roboticstoolbox-python (for multi-joint kinematics/dynamics)
