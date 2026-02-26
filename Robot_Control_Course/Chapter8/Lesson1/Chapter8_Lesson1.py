
import numpy as np

# True parameters (unknown to the controller)
m_true = 2.0   # inertia
b_true = 0.4   # viscous friction
g0_true = 5.0  # gravity coefficient

# Controller parameter estimates (to be adapted)
m_hat = 1.0
b_hat = 0.2
g0_hat = 3.0

k_p = 20.0
k_d = 8.0

def desired_trajectory(t):
    q_d = 0.5 * np.sin(0.5 * t)
    dq_d = 0.25 * np.cos(0.5 * t)
    ddq_d = -0.125 * np.sin(0.5 * t)
    return q_d, dq_d, ddq_d

def plant_dynamics(t, state, tau):
    q, dq = state
    ddq = (tau - b_true * dq - g0_true * np.sin(q)) / m_true
    return np.array([dq, ddq])

def control_law(t, q, dq, m_hat, b_hat, g0_hat):
    q_d, dq_d, ddq_d = desired_trajectory(t)
    e = q - q_d
    de = dq - dq_d

    # reference acceleration (as in computed torque)
    ddq_r = ddq_d - k_d * de - k_p * e
    dq_r = dq_d - k_p * e  # simple choice for illustration

    tau = m_hat * ddq_r + b_hat * dq_r + g0_hat * np.sin(q)
    return tau, e, de

def update_parameters(m_hat, b_hat, g0_hat, Y, tau, tau_hat, gamma=0.1):
    """
    Simple gradient update (for illustration only; rigorous laws in later lessons).
    tau: actual torque (what was commanded)
    tau_hat: Y theta_hat (model-predicted torque)
    """
    error = tau - tau_hat  # scalar
    grad = -Y * error      # gradient wrt theta_hat
    m_hat_new = m_hat - gamma * grad[0]
    b_hat_new = b_hat - gamma * grad[1]
    g0_hat_new = g0_hat - gamma * grad[2]
    return m_hat_new, b_hat_new, g0_hat_new

dt = 0.001
T = 5.0
steps = int(T / dt)

q = 0.0
dq = 0.0
history = []

for k in range(steps):
    t = k * dt
    tau, e, de = control_law(t, q, dq, m_hat, b_hat, g0_hat)

    # Regressor for this simple system
    q_d, dq_d, ddq_d = desired_trajectory(t)
    ddq_r = ddq_d - k_d * de - k_p * e
    dq_r = dq_d - k_p * e
    Y = np.array([ddq_r, dq_r, np.sin(q)])

    tau_hat = Y @ np.array([m_hat, b_hat, g0_hat])

    # Update parameter estimates (toy adaptation)
    m_hat, b_hat, g0_hat = update_parameters(m_hat, b_hat, g0_hat, Y, tau, tau_hat)

    # Integrate plant
    dq_mid = dq + 0.5 * dt * plant_dynamics(t, (q, dq), tau)[1]
    ddq_mid = (tau - b_true * dq_mid - g0_true * np.sin(q)) / m_true
    q += dq * dt
    dq += ddq_mid * dt

    history.append((t, q, dq, e, m_hat, b_hat, g0_hat))
