import numpy as np

# ---------- Encoder: counts -> angle, velocity ----------
def encoder_angle_velocity(counts, N, Ts):
    counts = np.asarray(counts, dtype=float)
    theta = (2*np.pi / N) * counts
    dtheta = np.diff(theta, prepend=theta[0]) / Ts
    return theta, dtheta

# Example
N = 2048
Ts = 0.001
counts = np.cumsum([0, 1, 1, 2, 2, 1, 0, -1])  # toy tick stream
theta_hat, omega_hat = encoder_angle_velocity(counts, N, Ts)

# ---------- IMU gyro integration (1D) ----------
def integrate_gyro(omega_meas, Ts, theta0=0.0):
    omega_meas = np.asarray(omega_meas, dtype=float)
    theta = np.zeros_like(omega_meas)
    theta[0] = theta0
    for k in range(1, len(omega_meas)):
        theta[k] = theta[k-1] + Ts * omega_meas[k-1]
    return theta

omega_meas = 0.2 + 0.01*np.random.randn(1000)  # rad/s with small noise
theta_from_gyro = integrate_gyro(omega_meas, Ts)

# ---------- F/T sensor: voltages -> wrench ----------
def wrench_from_voltages(v, S_hat, b_hat=None):
    v = np.asarray(v, dtype=float)
    if b_hat is not None:
        v = v - b_hat
    # Least-squares wrench estimate
    w_hat, *_ = np.linalg.lstsq(S_hat, v, rcond=None)
    return w_hat

m = 8  # number of gauge channels (example)
S_hat = np.random.randn(m, 6)
v_meas = np.random.randn(m)
w_hat = wrench_from_voltages(v_meas, S_hat)
