
import numpy as np
from math import sin, cos
from scipy.integrate import solve_ivp

# True parameters (plant)
theta_true = np.array([2.0, 0.4, 5.0])  # [theta1, theta2, theta3]

# Nominal model (fixed-model controller)
theta_nom = np.array([1.5, 0.2, 4.0])

# Adaptive initial parameters
theta_hat0 = theta_nom.copy()

# Gains
lam = 5.0
k_s = 10.0
Gamma = np.diag([5.0, 5.0, 5.0])  # adaptation gains

A = 0.5
omega = 1.0

def desired_traj(t):
    qd = A * np.sin(omega * t)
    dqd = A * omega * np.cos(omega * t)
    ddqd = -A * omega**2 * np.sin(omega * t)
    return qd, dqd, ddqd

def regressor(q, dq, qd, dqd, ddqd):
    e = q - qd
    de = dq - dqd
    dq_r = dqd - lam * e
    ddq_r = ddqd - lam * de
    Y = np.array([ddq_r, dq_r, np.sin(q)])  # shape (3,)
    s = dq - dq_r
    return Y, s

def plant_accel(q, dq, tau):
    theta1, theta2, theta3 = theta_true
    # theta1 * ddq + theta2 * dq + theta3 * sin(q) = tau
    return (tau - theta2 * dq - theta3 * np.sin(q)) / theta1

def rhs_fixed(t, x):
    # x = [q, dq]
    q, dq = x
    qd, dqd, ddqd = desired_traj(t)
    Y, s = regressor(q, dq, qd, dqd, ddqd)
    tau = Y.dot(theta_nom) - k_s * s
    ddq = plant_accel(q, dq, tau)
    return [dq, ddq]

def rhs_adaptive(t, x):
    # x = [q, dq, th1_hat, th2_hat, th3_hat]
    q, dq = x[0], x[1]
    theta_hat = x[2:5]
    qd, dqd, ddqd = desired_traj(t)
    Y, s = regressor(q, dq, qd, dqd, ddqd)
    tau = Y.dot(theta_hat) - k_s * s
    ddq = plant_accel(q, dq, tau)
    # adaptation law: dtheta_hat = -Gamma * Y^T * s
    dtheta_hat = -Gamma.dot(Y * s)
    return [dq, ddq, dtheta_hat[0], dtheta_hat[1], dtheta_hat[2]]

# Initial conditions
q0 = 0.0
dq0 = 0.0
x0_fixed = [q0, dq0]
x0_adapt = [q0, dq0, theta_hat0[0], theta_hat0[1], theta_hat0[2]]

t_span = (0.0, 20.0)
t_eval = np.linspace(t_span[0], t_span[1], 4001)

sol_fixed = solve_ivp(rhs_fixed, t_span, x0_fixed, t_eval=t_eval)
sol_adapt = solve_ivp(rhs_adaptive, t_span, x0_adapt, t_eval=t_eval)

# Post-processing: compute tracking error and control signals
q_fixed = sol_fixed.y[0]
q_adapt = sol_adapt.y[0]
t = t_eval

def compute_metrics(q_traj):
    e = np.zeros_like(q_traj)
    for i, ti in enumerate(t):
        qd, _, _ = desired_traj(ti)
        e[i] = q_traj[i] - qd
    rmse = np.sqrt(np.trapz(e**2, t) / (t[-1] - t[0]))
    return rmse, e

rmse_fixed, e_fixed = compute_metrics(q_fixed)
rmse_adapt, e_adapt = compute_metrics(q_adapt)

print("RMSE (fixed-model):", rmse_fixed)
print("RMSE (adaptive):   ", rmse_adapt)

# You can also reconstruct tau(t) by re-running regressor with stored states.
