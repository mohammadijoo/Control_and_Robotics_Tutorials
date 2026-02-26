# Chapter18_Lesson3.py
# Long-Range Localization Drift Handling (2D EKF with drift mitigation)
# State: [x, y, psi, b_g]^T
# - Propagation from wheel speed + gyro
# - GNSS/RTK updates with innovation gating
# - Terrain-aware process noise inflation
# - Anchor (map/loop-closure) re-localization updates
# - Simple integrity monitor and covariance inflation
#
# Requires: numpy

import numpy as np

np.random.seed(18)


def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def scalar_kf_update(x, P, idx, z, R, angle=False):
    """Kalman update with scalar measurement z = x[idx] + noise."""
    y = z - x[idx]
    if angle:
        y = wrap_angle(y)

    S = P[idx, idx] + R
    K = P[:, idx] / S
    x = x + K * y

    # Joseph-like simplified rank-1 covariance update
    P = P - np.outer(K, P[idx, :])
    P = 0.5 * (P + P.T)  # symmetry repair
    if angle:
        x[2] = wrap_angle(x[2])
    return x, P, float((y * y) / S)  # scalar NIS


def propagate(x, P, v_m, w_m, dt, rough=False):
    # Nominal propagation
    psi = x[2]
    bg = x[3]
    x_next = x.copy()
    x_next[0] += v_m * dt * np.cos(psi)
    x_next[1] += v_m * dt * np.sin(psi)
    x_next[2] = wrap_angle(x_next[2] + (w_m - bg) * dt)
    x_next[3] = x_next[3]

    # Linearized state transition
    F = np.eye(4)
    F[0, 2] = -v_m * dt * np.sin(psi)
    F[1, 2] =  v_m * dt * np.cos(psi)
    F[2, 3] = -dt

    # Process noise (terrain-dependent)
    sigma_v = 0.03 if not rough else 0.12
    sigma_w = 0.01 if not rough else 0.06
    sigma_bg_rw = 0.0008 if not rough else 0.0030

    qx = (sigma_v * dt) ** 2
    qy = (sigma_v * dt) ** 2
    qpsi = (sigma_w * dt) ** 2
    qbg = (sigma_bg_rw ** 2) * dt
    Q = np.diag([qx, qy, qpsi, qbg])

    P_next = F @ P @ F.T + Q
    P_next = 0.5 * (P_next + P_next.T)
    return x_next, P_next


# Mission setup
dt = 0.1
N = 1800
t = np.arange(N) * dt

# Truth state and sensor biases
x_true = np.zeros((N, 4))  # [x, y, psi, bg_true]
bg_true = 0.015  # rad/s gyro bias (unmodeled by pure dead reckoning)

# EKF estimate
x_est = np.array([0.0, 0.0, 0.05, 0.0])
P_est = np.diag([1.0, 1.0, (5*np.pi/180.0)**2, (0.03)**2])

# Storage
est_hist = np.zeros((N, 4))
P_trace = np.zeros(N)
nis_gnss = []
nis_anchor = []

# Outages and anchor events
gnss_outage_1 = (420, 980)
gnss_outage_2 = (1250, 1500)
anchor_times = {1000, 1510, 1710}  # pseudo loop-closure / map-anchor moments

# Simulate + filter
for k in range(1, N):
    # Truth command profile (long-range path with turns)
    tk = t[k]
    v_cmd = 1.5 + 0.3 * np.sin(0.015 * tk)
    w_cmd = 0.08 * np.sin(0.010 * tk) + 0.04 * np.sin(0.040 * tk)

    # "Rough terrain" intervals (slip/vibration increase)
    rough = (300 <= k < 520) or (840 <= k < 1100) or (1400 <= k < 1620)

    # True propagation
    sigma_w_true = 0.008 if not rough else 0.03
    sigma_v_true = 0.02 if not rough else 0.10
    v_true = v_cmd + np.random.randn() * sigma_v_true
    w_true = w_cmd + np.random.randn() * sigma_w_true

    x_true[k, 0] = x_true[k-1, 0] + v_true * dt * np.cos(x_true[k-1, 2])
    x_true[k, 1] = x_true[k-1, 1] + v_true * dt * np.sin(x_true[k-1, 2])
    x_true[k, 2] = wrap_angle(x_true[k-1, 2] + (w_true - bg_true) * dt)
    x_true[k, 3] = bg_true

    # Sensor measurements (wheel speed + gyro)
    v_m = v_true + np.random.randn() * (0.03 if not rough else 0.12)
    w_m = (w_true - bg_true) + bg_true + np.random.randn() * (0.01 if not rough else 0.05)
    # (equivalently w_m = w_true + noise, but model estimates bg)

    # EKF propagation
    x_est, P_est = propagate(x_est, P_est, v_m, w_m, dt, rough=rough)

    # GNSS/RTK update (degrades or disappears)
    in_outage = (gnss_outage_1[0] <= k < gnss_outage_1[1]) or (gnss_outage_2[0] <= k < gnss_outage_2[1])
    if (k % 10 == 0) and (not in_outage):
        # RTK fixed/float mode emulation
        rtk_fixed = not rough and (k % 90 != 0)
        sigma_gps = 0.15 if rtk_fixed else 0.75
        zgx = x_true[k, 0] + np.random.randn() * sigma_gps
        zgy = x_true[k, 1] + np.random.randn() * sigma_gps

        # Sequential scalar gating (x then y); 99% chi-square for 1 dof ~ 6.63
        x_pred = x_est.copy()
        P_pred = P_est.copy()

        # x measurement
        innov_x = zgx - x_pred[0]
        Sx = P_pred[0, 0] + sigma_gps**2
        d2x = innov_x**2 / Sx
        if d2x < 6.63:
            x_est, P_est, nisx = scalar_kf_update(x_est, P_est, 0, zgx, sigma_gps**2, angle=False)
            nis_gnss.append(nisx)

        # y measurement
        innov_y = zgy - x_est[1]
        Sy = P_est[1, 1] + sigma_gps**2
        d2y = innov_y**2 / Sy
        if d2y < 6.63:
            x_est, P_est, nisy = scalar_kf_update(x_est, P_est, 1, zgy, sigma_gps**2, angle=False)
            nis_gnss.append(nisy)

    # Anchor re-localization update (e.g., scan-map/VIO-map loop closure)
    if k in anchor_times:
        z_anchor = np.array([
            x_true[k, 0] + np.random.randn() * 0.10,
            x_true[k, 1] + np.random.randn() * 0.10,
            wrap_angle(x_true[k, 2] + np.random.randn() * (2.0*np.pi/180.0))
        ])

        # Sequential x, y, psi anchor update with tight covariance
        x_est, P_est, n1 = scalar_kf_update(x_est, P_est, 0, z_anchor[0], 0.10**2, angle=False)
        x_est, P_est, n2 = scalar_kf_update(x_est, P_est, 1, z_anchor[1], 0.10**2, angle=False)
        x_est, P_est, n3 = scalar_kf_update(x_est, P_est, 2, z_anchor[2], (2.0*np.pi/180.0)**2, angle=True)
        nis_anchor.extend([n1, n2, n3])

    # Integrity monitor: inflate if innovation statistics remain high
    if len(nis_gnss) >= 20:
        window_mean = float(np.mean(nis_gnss[-20:]))
        if window_mean > 1.8:  # expected mean ~= 1 for scalar NIS
            P_est *= 1.02

    est_hist[k] = x_est
    P_trace[k] = np.trace(P_est)

# Metrics
pos_err = est_hist[:, :2] - x_true[:, :2]
rmse = np.sqrt(np.mean(np.sum(pos_err**2, axis=1)))
final_err = np.linalg.norm(pos_err[-1])
heading_err_deg = np.rad2deg(wrap_angle(est_hist[-1, 2] - x_true[-1, 2]))

print("=== Chapter18_Lesson3.py : Long-Range Localization Drift Handling ===")
print(f"Mission duration: {N*dt:.1f} s")
print(f"Position RMSE: {rmse:.3f} m")
print(f"Final position error: {final_err:.3f} m")
print(f"Final heading error: {heading_err_deg:.3f} deg")
print(f"Mean scalar NIS (GNSS accepted): {np.mean(nis_gnss) if nis_gnss else np.nan:.3f}")
print(f"Mean scalar NIS (Anchor): {np.mean(nis_anchor) if nis_anchor else np.nan:.3f}")
print(f"Final covariance trace: {P_trace[-1]:.5f}")

# A tiny textual drift summary across outage segments
for label, (a, b) in {"Outage1": gnss_outage_1, "Outage2": gnss_outage_2}.items():
    e_start = np.linalg.norm(est_hist[a, :2] - x_true[a, :2])
    e_end = np.linalg.norm(est_hist[b-1, :2] - x_true[b-1, :2])
    print(f"{label}: error at start={e_start:.3f} m, end={e_end:.3f} m, growth={e_end-e_start:.3f} m")
