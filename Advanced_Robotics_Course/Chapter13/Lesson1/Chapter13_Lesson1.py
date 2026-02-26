import numpy as np

# Discrete-time 1-DOF dynamics parameters
dt = 0.02
c_sim = 0.1      # damping used during "training"
c_real = 0.3     # true damping on the robot (unknown to the designer)

# Linear feedback policy u = -K x, where x = [theta, omega]
Kp, Kd = 5.0, 1.0

def policy(x):
    theta, omega = x
    return -Kp * theta - Kd * omega

def step(x, u, c):
    theta, omega = x
    theta_next = theta + dt * omega
    omega_next = omega + dt * (u - c * omega)
    return np.array([theta_next, omega_next])

def rollout(c, horizon=500):
    x = np.array([0.5, 0.0])  # start with a position error
    total_reward = 0.0
    gamma = 0.99
    for t in range(horizon):
        u = policy(x)
        # quadratic cost; reward = -cost
        cost = x[0]**2 + 0.1 * x[1]**2 + 0.01 * u**2
        total_reward += (gamma**t) * (-cost)
        x = step(x, u, c)
    return total_reward

J_sim = rollout(c_sim)
J_real = rollout(c_real)

print("Discounted return in simulation:", J_sim)
print("Discounted return on 'real' model:", J_real)
print("Sim-to-real gap:", J_real - J_sim)
      
