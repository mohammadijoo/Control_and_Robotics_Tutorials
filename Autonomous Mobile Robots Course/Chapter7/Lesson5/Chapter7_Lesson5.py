# Chapter7_Lesson5.py
# EKF/UKF localization lab: Wheel + IMU + GPS (2D ground robot)
# Dependencies: numpy, matplotlib
# Optional: scipy (not required)

import math
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(7)

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def angle_mean(angles: np.ndarray, w: np.ndarray) -> float:
    """Weighted circular mean."""
    s = np.sum(w * np.sin(angles))
    c = np.sum(w * np.cos(angles))
    return float(np.arctan2(s, c))

def angle_residual(a: float, b: float) -> float:
    """Residual a - b for angles."""
    return wrap_angle(a - b)

# -------------------------- Simulation --------------------------

def simulate_truth_and_sensors(T=60.0, dt_imu=0.01, dt_wheel=0.05, dt_gps=1.0):
    """
    Simulate a planar trajectory and noisy sensors:
      - IMU: yaw rate omega_m and longitudinal accel a_m at dt_imu
      - Wheel: forward speed v_w and yaw rate omega_w at dt_wheel
      - GPS: position (x_gps, y_gps) at dt_gps
    State (truth):
      x, y, theta, v, b_g, b_a
    """
    n_imu = int(T / dt_imu) + 1
    t = np.linspace(0.0, T, n_imu)

    # True biases (slowly varying via random walk)
    bg0, ba0 = 0.02, -0.05  # gyro bias rad/s, accel bias m/s^2

    # Noise parameters (tune for your platform)
    sigma_g = 0.02          # gyro white noise (rad/s)
    sigma_a = 0.20          # accel white noise (m/s^2)
    sigma_bg_rw = 5e-4      # gyro-bias RW (rad/s/sqrt(s))
    sigma_ba_rw = 2e-3      # accel-bias RW (m/s^2/sqrt(s))

    sigma_vw = 0.10         # wheel speed noise (m/s)
    sigma_ww = 0.02         # wheel yaw-rate noise (rad/s)

    sigma_gps = 1.5         # GPS position noise (m)

    # Allocate arrays
    X_true = np.zeros((n_imu, 6))  # [x, y, theta, v, bg, ba]
    omega_true = np.zeros(n_imu)
    a_true = np.zeros(n_imu)

    # Initial truth
    X_true[0, :] = np.array([0.0, 0.0, 0.2, 1.0, bg0, ba0])

    # Trajectory profile (smooth turns + speed changes)
    for k in range(n_imu - 1):
        tt = t[k]
        # commanded (true) yaw-rate and accel profiles
        omega_cmd = 0.25 * np.sin(0.2 * tt) + 0.05 * np.sin(1.1 * tt)
        a_cmd = 0.4 * np.sin(0.1 * tt)

        omega_true[k] = omega_cmd
        a_true[k] = a_cmd

        x, y, th, v, bg, ba = X_true[k, :]

        # Bias random walk
        bg_next = bg + sigma_bg_rw * np.sqrt(dt_imu) * np.random.randn()
        ba_next = ba + sigma_ba_rw * np.sqrt(dt_imu) * np.random.randn()

        # Dynamics (Euler)
        th_next = wrap_angle(th + (omega_cmd) * dt_imu)
        v_next = v + (a_cmd) * dt_imu
        x_next = x + v * dt_imu * np.cos(th)
        y_next = y + v * dt_imu * np.sin(th)

        X_true[k + 1, :] = np.array([x_next, y_next, th_next, v_next, bg_next, ba_next])

    omega_true[-1] = omega_true[-2]
    a_true[-1] = a_true[-2]

    # Generate sensors
    imu_omega_m = omega_true + X_true[:, 4] + sigma_g * np.random.randn(n_imu)
    imu_a_m = a_true + X_true[:, 5] + sigma_a * np.random.randn(n_imu)

    # Wheel / GPS sample indices
    wheel_idx = np.arange(0, n_imu, int(round(dt_wheel / dt_imu)))
    gps_idx = np.arange(0, n_imu, int(round(dt_gps / dt_imu)))

    wheel_v = X_true[wheel_idx, 3] + sigma_vw * np.random.randn(len(wheel_idx))
    wheel_omega = omega_true[wheel_idx] + sigma_ww * np.random.randn(len(wheel_idx))

    gps_xy = X_true[gps_idx, 0:2] + sigma_gps * np.random.randn(len(gps_idx), 2)

    sensors = {
        "t": t,
        "imu_omega_m": imu_omega_m,
        "imu_a_m": imu_a_m,
        "wheel_idx": wheel_idx,
        "wheel_v": wheel_v,
        "wheel_omega": wheel_omega,
        "gps_idx": gps_idx,
        "gps_xy": gps_xy,
        "params": {
            "dt_imu": dt_imu,
            "dt_wheel": dt_wheel,
            "dt_gps": dt_gps,
            "sigma_g": sigma_g,
            "sigma_a": sigma_a,
            "sigma_bg_rw": sigma_bg_rw,
            "sigma_ba_rw": sigma_ba_rw,
            "sigma_vw": sigma_vw,
            "sigma_ww": sigma_ww,
            "sigma_gps": sigma_gps,
        }
    }
    return X_true, sensors

# -------------------------- EKF --------------------------

class EKF:
    """
    EKF with state x = [x, y, theta, v, b_g, b_a]^T
    IMU (omega_m, a_m) used in the process model (as inputs).
    Wheel provides measurement of v and yaw-rate (omega_w).
    GPS provides measurement of x,y.
    """
    def __init__(self, x0: np.ndarray, P0: np.ndarray, Qc: np.ndarray, Rw: np.ndarray, Rg: np.ndarray):
        self.x = x0.astype(float).copy()
        self.P = P0.astype(float).copy()
        self.Qc = Qc.astype(float).copy()  # continuous-time noise cov for [n_g, n_a, w_bg, w_ba]
        self.Rw = Rw.astype(float).copy()
        self.Rg = Rg.astype(float).copy()

    def predict(self, omega_m: float, a_m: float, dt: float):
        x, y, th, v, bg, ba = self.x

        omega = omega_m - bg
        acc = a_m - ba

        # Nonlinear propagation
        x_pred = x + v * dt * math.cos(th)
        y_pred = y + v * dt * math.sin(th)
        th_pred = wrap_angle(th + omega * dt)
        v_pred = v + acc * dt
        bg_pred = bg
        ba_pred = ba

        self.x = np.array([x_pred, y_pred, th_pred, v_pred, bg_pred, ba_pred])

        # Jacobian F = df/dx
        F = np.eye(6)
        F[0, 2] = -v * dt * math.sin(th)
        F[0, 3] = dt * math.cos(th)
        F[1, 2] =  v * dt * math.cos(th)
        F[1, 3] = dt * math.sin(th)
        F[2, 4] = -dt
        F[3, 5] = -dt

        # Noise mapping G for w = [n_g, n_a, w_bg, w_ba]
        # theta affected by gyro noise n_g; v affected by accel noise n_a; bg, ba random walk
        G = np.zeros((6, 4))
        G[2, 0] = dt
        G[3, 1] = dt
        G[4, 2] = math.sqrt(dt)
        G[5, 3] = math.sqrt(dt)

        Qd = G @ self.Qc @ G.T  # simple discretization

        self.P = F @ self.P @ F.T + Qd

    def update_wheel(self, v_w: float, omega_w: float, omega_m: float):
        """
        z_w = [v_w, omega_w]^T
        h_w(x, u) = [v, (omega_m - b_g)]^T
        """
        x, y, th, v, bg, ba = self.x
        z = np.array([v_w, omega_w])
        h = np.array([v, omega_m - bg])

        H = np.zeros((2, 6))
        H[0, 3] = 1.0
        H[1, 4] = -1.0

        self._update(z, h, H, self.Rw, angle_rows=None)

    def update_gps(self, xy_gps: np.ndarray):
        z = xy_gps.reshape(2)
        h = self.x[0:2].copy()

        H = np.zeros((2, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        self._update(z, h, H, self.Rg, angle_rows=None)

    def _update(self, z, h, H, R, angle_rows=None):
        y = z - h
        # If any measurement component is angular, wrap residuals (not used here)
        if angle_rows is not None:
            for r in angle_rows:
                y[r] = wrap_angle(y[r])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        dx = K @ y
        self.x = self.x + dx
        self.x[2] = wrap_angle(self.x[2])

        I = np.eye(6)
        # Joseph form (PSD-safe)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

# -------------------------- UKF --------------------------

class UKF:
    """
    UKF with same state definition as EKF.
    Uses a minimal circular-mean handling for theta.
    """
    def __init__(self, x0: np.ndarray, P0: np.ndarray, Qd: np.ndarray, Rw: np.ndarray, Rg: np.ndarray,
                 alpha=1e-3, beta=2.0, kappa=0.0):
        self.x = x0.astype(float).copy()
        self.P = P0.astype(float).copy()
        self.Qd = Qd.astype(float).copy()
        self.Rw = Rw.astype(float).copy()
        self.Rg = Rg.astype(float).copy()

        self.n = 6
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lmbda = alpha**2 * (self.n + kappa) - self.n

        self.Wm = np.full(2 * self.n + 1, 1.0 / (2.0 * (self.n + self.lmbda)))
        self.Wc = self.Wm.copy()
        self.Wm[0] = self.lmbda / (self.n + self.lmbda)
        self.Wc[0] = self.Wm[0] + (1.0 - alpha**2 + beta)

    def _sigma_points(self, x: np.ndarray, P: np.ndarray):
        S = np.linalg.cholesky((self.n + self.lmbda) * P)
        X = np.zeros((2 * self.n + 1, self.n))
        X[0, :] = x
        for i in range(self.n):
            X[i + 1, :] = x + S[:, i]
            X[i + 1 + self.n, :] = x - S[:, i]
        # wrap theta coordinate in sigma points
        X[:, 2] = np.vectorize(wrap_angle)(X[:, 2])
        return X

    def _f(self, x, omega_m, a_m, dt):
        x0, y0, th, v, bg, ba = x
        omega = omega_m - bg
        acc = a_m - ba
        x1 = x0 + v * dt * math.cos(th)
        y1 = y0 + v * dt * math.sin(th)
        th1 = wrap_angle(th + omega * dt)
        v1 = v + acc * dt
        return np.array([x1, y1, th1, v1, bg, ba])

    def predict(self, omega_m: float, a_m: float, dt: float):
        X = self._sigma_points(self.x, self.P)
        Xp = np.zeros_like(X)
        for i in range(X.shape[0]):
            Xp[i, :] = self._f(X[i, :], omega_m, a_m, dt)

        # Mean with special theta handling
        x_mean = np.sum(self.Wm[:, None] * Xp, axis=0)
        x_mean[2] = angle_mean(Xp[:, 2], self.Wm)

        # Covariance
        P = np.zeros((self.n, self.n))
        for i in range(Xp.shape[0]):
            dx = Xp[i, :] - x_mean
            dx[2] = angle_residual(Xp[i, 2], x_mean[2])
            P += self.Wc[i] * np.outer(dx, dx)
        P += self.Qd

        self.x = x_mean
        self.P = P

    def update_wheel(self, v_w: float, omega_w: float, omega_m: float):
        # z = [v, omega_m - bg] + noise
        z = np.array([v_w, omega_w])

        def h(x):
            return np.array([x[3], omega_m - x[4]])

        self._update(z, h, self.Rw, angle_meas_idx=None)

    def update_gps(self, xy_gps: np.ndarray):
        z = xy_gps.reshape(2)

        def h(x):
            return x[0:2]

        self._update(z, h, self.Rg, angle_meas_idx=None)

    def _update(self, z, h, R, angle_meas_idx=None):
        X = self._sigma_points(self.x, self.P)
        Z = np.array([h(X[i, :]) for i in range(X.shape[0])])
        m = Z.shape[1]

        z_mean = np.sum(self.Wm[:, None] * Z, axis=0)
        # if angle measurements exist, implement circular mean (not needed here)

        S = np.zeros((m, m))
        Pxz = np.zeros((self.n, m))
        for i in range(Z.shape[0]):
            dz = Z[i, :] - z_mean
            dx = X[i, :] - self.x
            dx[2] = angle_residual(X[i, 2], self.x[2])
            S += self.Wc[i] * np.outer(dz, dz)
            Pxz += self.Wc[i] * np.outer(dx, dz)
        S += R

        K = Pxz @ np.linalg.inv(S)
        innov = z - z_mean
        self.x = self.x + K @ innov
        self.x[2] = wrap_angle(self.x[2])
        self.P = self.P - K @ S @ K.T

# -------------------------- Run lab --------------------------

def run():
    X_true, sensors = simulate_truth_and_sensors(T=80.0, dt_imu=0.01, dt_wheel=0.05, dt_gps=1.0)
    t = sensors["t"]
    dt = sensors["params"]["dt_imu"]

    # Initial guess (intentionally imperfect)
    x0 = np.array([0.5, -1.0, 0.0, 0.5, 0.0, 0.0])
    P0 = np.diag([4.0, 4.0, (20.0*np.pi/180.0)**2, 1.0, 0.05**2, 0.2**2])

    # Continuous noise cov for EKF discretization (n_g, n_a, w_bg, w_ba)
    sigma_g = sensors["params"]["sigma_g"]
    sigma_a = sensors["params"]["sigma_a"]
    sigma_bg_rw = sensors["params"]["sigma_bg_rw"]
    sigma_ba_rw = sensors["params"]["sigma_ba_rw"]

    Qc = np.diag([sigma_g**2, sigma_a**2, sigma_bg_rw**2, sigma_ba_rw**2])

    # Measurement covariances
    sigma_vw = sensors["params"]["sigma_vw"]
    sigma_ww = sensors["params"]["sigma_ww"]
    sigma_gps = sensors["params"]["sigma_gps"]

    Rw = np.diag([sigma_vw**2, sigma_ww**2])
    Rg = np.diag([sigma_gps**2, sigma_gps**2])

    ekf = EKF(x0, P0, Qc, Rw, Rg)

    # UKF: use a simple discrete Qd (tuned similarly)
    # We approximate Qd at dt with diagonal injection for theta,v,bg,ba components.
    Qd = np.diag([0.0, 0.0, (sigma_g*dt)**2, (sigma_a*dt)**2,
                 (sigma_bg_rw*np.sqrt(dt))**2, (sigma_ba_rw*np.sqrt(dt))**2])
    ukf = UKF(x0, P0, Qd, Rw, Rg)

    wheel_map = {idx: i for i, idx in enumerate(sensors["wheel_idx"])}
    gps_map = {idx: i for i, idx in enumerate(sensors["gps_idx"])}

    X_ekf = np.zeros_like(X_true)
    X_ukf = np.zeros_like(X_true)

    for k in range(len(t)):
        omega_m = float(sensors["imu_omega_m"][k])
        a_m = float(sensors["imu_a_m"][k])

        # Predict at IMU rate
        ekf.predict(omega_m, a_m, dt)
        ukf.predict(omega_m, a_m, dt)

        # Wheel update (lower rate)
        if k in wheel_map:
            j = wheel_map[k]
            ekf.update_wheel(float(sensors["wheel_v"][j]), float(sensors["wheel_omega"][j]), omega_m)
            ukf.update_wheel(float(sensors["wheel_v"][j]), float(sensors["wheel_omega"][j]), omega_m)

        # GPS update (lowest rate)
        if k in gps_map:
            j = gps_map[k]
            ekf.update_gps(sensors["gps_xy"][j])
            ukf.update_gps(sensors["gps_xy"][j])

        X_ekf[k, :] = ekf.x
        X_ukf[k, :] = ukf.x

    # Metrics
    def rmse(a, b):
        return np.sqrt(np.mean((a - b)**2))

    pos_rmse_ekf = rmse(X_ekf[:, 0:2], X_true[:, 0:2])
    pos_rmse_ukf = rmse(X_ukf[:, 0:2], X_true[:, 0:2])
    th_rmse_ekf = np.sqrt(np.mean(np.array([angle_residual(X_ekf[i,2], X_true[i,2]) for i in range(len(t))])**2))
    th_rmse_ukf = np.sqrt(np.mean(np.array([angle_residual(X_ukf[i,2], X_true[i,2]) for i in range(len(t))])**2))

    print("Position RMSE (m): EKF =", pos_rmse_ekf, " UKF =", pos_rmse_ukf)
    print("Heading RMSE (rad): EKF =", th_rmse_ekf, " UKF =", th_rmse_ukf)

    # Plots
    plt.figure()
    plt.plot(X_true[:,0], X_true[:,1], label="true")
    plt.plot(X_ekf[:,0], X_ekf[:,1], label="EKF")
    plt.plot(X_ukf[:,0], X_ukf[:,1], label="UKF")
    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()
    plt.title("Trajectory")

    plt.figure()
    plt.plot(t, X_true[:,2], label="true")
    plt.plot(t, X_ekf[:,2], label="EKF")
    plt.plot(t, X_ukf[:,2], label="UKF")
    plt.xlabel("t (s)")
    plt.ylabel("theta (rad)")
    plt.legend()
    plt.title("Heading")

    plt.figure()
    plt.plot(t, X_true[:,3], label="true")
    plt.plot(t, X_ekf[:,3], label="EKF")
    plt.plot(t, X_ukf[:,3], label="UKF")
    plt.xlabel("t (s)")
    plt.ylabel("v (m/s)")
    plt.legend()
    plt.title("Speed")

    plt.show()

if __name__ == "__main__":
    run()
