import numpy as np
from math import copysign

# Example motor parameters (normalized)
K = 10.0       # dc gain
tau_m = 0.05   # mechanical time constant
Ts = 0.001     # sampling period (1 ms)

# PID gains
Kp = 2.0
Ki = 50.0
Kd = 0.001
alpha = 0.9    # derivative filter coefficient
K_aw = 20.0    # anti-windup gain

u_min, u_max = -12.0, 12.0  # actuator voltage limits

# Simulation horizon
T = 0.5
N = int(T / Ts)

# Allocate arrays
t = np.linspace(0.0, T, N+1)
theta = np.zeros(N+1)      # position
theta_dot = np.zeros(N+1)  # velocity (state)
u = np.zeros(N+1)          # control
r = np.ones(N+1) * 1.0     # 1 radian step

# PID internal states
I = 0.0
D = 0.0
e_prev = 0.0

for k in range(N):
    # Error
    e = r[k] - theta[k]

    # Derivative filter
    D = alpha * D + (1.0 - alpha) * (e - e_prev) / Ts

    # Unsaturated PID output
    v = Kp * e + I + Kd * D

    # Saturation
    u[k] = max(u_min, min(u_max, v))

    # Anti-windup back-calculation
    I = I + Ki * Ts * e + K_aw * (u[k] - v)

    # Motor dynamics: theta_dot_dot = -(1/tau_m)*theta_dot + (K/tau_m)*u
    theta_ddot = -(1.0 / tau_m) * theta_dot[k] + (K / tau_m) * u[k]

    # Integrate dynamics with simple Euler
    theta_dot[k+1] = theta_dot[k] + Ts * theta_ddot
    theta[k+1] = theta[k] + Ts * theta_dot[k+1]

    e_prev = e

# The arrays t, theta, u now contain the simulated response.
# For robotics contexts, one may integrate this PID loop into a ROS node:
# - Use sensor_msgs/JointState for theta
# - Publish u as an effort command through control_msgs/JointJog or similar.
