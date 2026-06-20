import numpy as np

# Robot joint approximate first-order dynamics: dy/dt = (-1/Tp)*y + (Kp/Tp)*u
Tp = 0.15  # process time constant [s]
Kp_proc = 1.0
u_min, u_max = -5.0, 5.0  # actuator limits (e.g. voltage)

Kp = 8.0   # proportional gain
Ki = 20.0  # integral gain
T_aw = 0.05  # anti-windup time constant

Ts = 0.001
t_final = 1.5
n_steps = int(t_final / Ts)

r = np.ones(n_steps) * 1.0  # desired joint angle [rad], step to 1.0
y = np.zeros(n_steps)       # measured joint angle
u = np.zeros(n_steps)       # actuator command
v = np.zeros(n_steps)       # unsaturated controller output
xI = 0.0                    # integral state

def sat(val, u_min, u_max):
    return max(u_min, min(u_max, val))

for k in range(n_steps - 1):
    # error
    e = r[k] - y[k]

    # PI control with back-calculation
    # Predict integral update using current error and AW correction
    v[k] = Kp * e + Ki * xI
    u[k] = sat(v[k], u_min, u_max)

    # anti-windup correction term
    e_aw = u[k] - v[k]
    xI = xI + Ts * (e + (1.0 / T_aw) * e_aw)

    # plant update (Euler integration)
    dy = (-1.0 / Tp) * y[k] + (Kp_proc / Tp) * u[k]
    y[k + 1] = y[k] + Ts * dy

# At this point, vectors r, y, u, v contain the simulation results.
# In a robotics context you could publish 'u[k]' at each step via a ROS topic,
# or implement the same loop inside a real-time control thread.
