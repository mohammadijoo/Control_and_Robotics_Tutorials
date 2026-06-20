import numpy as np

# Controller parameters from design
Kp = 3.2
Ki = 9.0

# Simulation parameters
dt = 0.001     # time step [s]
t_final = 8.0  # final time [s]
n_steps = int(t_final / dt) + 1

# Allocate arrays
t = np.linspace(0.0, t_final, n_steps)
y = np.zeros(n_steps)   # plant output
z = np.zeros(n_steps)   # integral state
u = np.zeros(n_steps)   # control signal
d = np.zeros(n_steps)   # disturbance

# Reference: unit step
def reference(time):
    return 1.0

# Disturbance: step applied at t >= 3 s
def disturbance(time):
    return 0.2 if time >= 3.0 else 0.0

for k in range(n_steps - 1):
    r_k = reference(t[k])
    d_k = disturbance(t[k])
    e_k = r_k - y[k]

    # PI control law
    u[k] = Kp * e_k + Ki * z[k]

    # Integral state derivative
    dz = e_k

    # Plant: y_dot = -y + u + d
    dy = -y[k] + u[k] + d_k

    # Euler integration
    z[k + 1] = z[k] + dt * dz
    y[k + 1] = y[k] + dt * dy

# Last control and disturbance
u[-1] = Kp * (reference(t[-1]) - y[-1]) + Ki * z[-1]
d[-1] = disturbance(t[-1])

# At this point, t, y, u, d contain the closed-loop trajectory.
# In a robotic joint controller, similar logic would be embedded
# in a 1 kHz control loop using python-control or roboticstoolbox.
