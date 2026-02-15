
import numpy as np

# Robot joint parameters (small-angle linearized)
J = 0.05   # inertia
d = 0.01   # viscous friction
Kp = 50.0
Kd = 2.0 * np.sqrt(Kp * J)  # critical-ish damping

# Discretization
dt = 0.001   # controller/estimator step (1 kHz)
h  = 0.02    # encoder delay = 20 ms
delay_steps = int(h / dt)

# State: x = [q, qd, b]^T
A = np.array([[0.0, 1.0, 0.0],
              [0.0, -d / J, 0.0],
              [0.0, 0.0, 0.0]])
B = np.array([[0.0],
              [1.0 / J],
              [0.0]])
C = np.array([[1.0, 0.0, 1.0]])  # y = q + b + noise

# EKF covariances (tuned)
Q = np.diag([1e-6, 1e-4, 1e-8])   # process noise
R = np.array([[1e-4]])            # meas noise

# Reference trajectory
def q_ref(t):
    return 0.5 * np.sin(2.0 * np.pi * 0.5 * t)  # 0.5 rad at 0.5 Hz

# Circular buffer for delayed measurements
max_steps = 50000
meas_buffer = [None] * (delay_steps + 1)

# Initialization
x_true = np.array([0.0, 0.0, 0.05])  # true bias 0.05 rad
x_hat  = np.array([0.0, 0.0, 0.0])   # initial bias estimate 0
P = np.eye(3) * 1e-2

def step_true(x, u):
    # exact discrete-time update via Euler for simplicity
    return x + dt * (A @ x + B.flatten() * u)

def ekf_predict(xh, Ph, u):
    Ad = np.eye(3) + dt * A
    Bd = dt * B
    xh = Ad @ xh + Bd.flatten() * u
    Ph = Ad @ Ph @ Ad.T + Q
    return xh, Ph

def ekf_update(xh, Ph, y):
    S = C @ Ph @ C.T + R
    K = Ph @ C.T @ np.linalg.inv(S)
    innov = y - C @ xh
    xh = xh + (K @ innov).flatten()
    Ph = (np.eye(3) - K @ C) @ Ph
    return xh, Ph

T_final = 10.0
N = int(T_final / dt)
t = 0.0

q_log = []
q_hat_log = []
b_hat_log = []

for k in range(N):
    # reference at current time
    qr = q_ref(t)
    # PD control using estimated state
    e = qr - x_hat[0]
    ed = 0.0 - x_hat[1]
    u = Kp * e + Kd * ed

    # plant propagation
    x_true = step_true(x_true, u)

    # generate measurement (delayed)
    y_now = x_true[0] + x_true[2] + np.random.randn() * np.sqrt(R[0, 0])
    meas_buffer[k % len(meas_buffer)] = y_now

    # EKF predict
    x_hat, P = ekf_predict(x_hat, P, u)

    # EKF update with delayed measurement (if available)
    # Here we approximate alignment by feeding measurement produced h seconds ago.
    idx_delay = (k - delay_steps) % len(meas_buffer)
    if k >= delay_steps:
        y_delayed = meas_buffer[idx_delay]
        x_hat, P = ekf_update(x_hat, P, y_delayed)

    # logs
    q_log.append(x_true[0])
    q_hat_log.append(x_hat[0])
    b_hat_log.append(x_hat[2])
    t += dt

print("Final bias estimate b_hat:", b_hat_log[-1])
