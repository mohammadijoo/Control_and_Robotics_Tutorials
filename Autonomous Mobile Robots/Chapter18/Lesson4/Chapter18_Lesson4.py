# Chapter18_Lesson4.py
# Weather/Lighting Effects on Perception (Outdoor and Field AMR)
# This script demonstrates:
# 1) A simple fog + low-light camera degradation model
# 2) Quality metrics (contrast / gradient energy proxy)
# 3) Confidence-aware EKF measurement covariance inflation for camera/LiDAR updates
#
# Related robotics stacks for deployment:
# - ROS 2 (sensor_msgs, image_transport, nav_msgs)
# - OpenCV (camera preprocessing), PCL (point-cloud filtering)
# - robot_localization / custom EKF nodes

import numpy as np
import matplotlib.pyplot as plt

rng = np.random.default_rng(7)

# ------------------------------------------------------------
# Part A: Synthetic radiometric degradation (1-D signal proxy)
# ------------------------------------------------------------
n = 400
x = np.linspace(0.0, 1.0, n)

# "Clear" reflectance-like profile with edges and texture
J = 0.25 + 0.35 * (x > 0.2) + 0.15 * np.sin(14 * np.pi * x)
J += 0.20 * (x > 0.58)
J = np.clip(J, 0.0, 1.0)

# Depth profile (farther on the right side)
d = 5.0 + 30.0 * x  # meters
beta = 0.07         # fog extinction coefficient [1/m]
A = 0.85            # atmospheric light

t = np.exp(-beta * d)
I_fog = J * t + A * (1.0 - t)

# Low-light + shot/read noise proxy
exposure_scale = 0.28
read_sigma = 0.015
shot_sigma = np.sqrt(np.maximum(I_fog * exposure_scale, 1e-6)) * 0.05
I_low = exposure_scale * I_fog + rng.normal(0.0, shot_sigma + read_sigma, size=n)
I_low = np.clip(I_low, 0.0, 1.0)

# Simple "illumination compensation" and contrast normalization
gamma = 0.6
I_enh = np.power(np.clip(I_low, 0.0, 1.0), gamma)
I_enh = (I_enh - I_enh.min()) / (I_enh.max() - I_enh.min() + 1e-12)

def michelson_contrast(sig):
    smax = float(np.max(sig))
    smin = float(np.min(sig))
    return (smax - smin) / (smax + smin + 1e-12)

def gradient_energy(sig):
    g = np.diff(sig)
    return float(np.mean(g * g))

print("=== Signal Quality Metrics ===")
print(f"Michelson contrast (clear): {michelson_contrast(J):.4f}")
print(f"Michelson contrast (fog+low-light): {michelson_contrast(I_low):.4f}")
print(f"Michelson contrast (enhanced): {michelson_contrast(I_enh):.4f}")
print(f"Gradient energy (clear): {gradient_energy(J):.6f}")
print(f"Gradient energy (fog+low-light): {gradient_energy(I_low):.6f}")
print(f"Gradient energy (enhanced): {gradient_energy(I_enh):.6f}")

# ------------------------------------------------------------
# Part B: Confidence-aware EKF (2-state constant velocity model)
# ------------------------------------------------------------
dt = 0.1
T = 250
F = np.array([[1.0, dt],
              [0.0, 1.0]])
Q = np.array([[1e-4, 0.0],
              [0.0, 3e-3]])

H_cam = np.array([[1.0, 0.0]])
H_lidar = np.array([[1.0, 0.0]])

R_cam0 = np.array([[0.6**2]])
R_lidar0 = np.array([[0.25**2]])

def weather_profile(k):
    fog = 0.10 + 0.35 * np.exp(-0.5 * ((k - 120) / 22.0)**2)
    lux = 1.0 if k < 140 else 0.25
    rain = 0.05 + 0.25 * np.exp(-0.5 * ((k - 180) / 14.0)**2)
    return fog, lux, rain

def camera_confidence(fog, lux, rain):
    q = (lux**0.7) * np.exp(-2.2 * fog) * np.exp(-1.4 * rain)
    return float(np.clip(q, 0.05, 1.0))

def lidar_confidence(fog, rain):
    q = np.exp(-1.3 * fog) * np.exp(-0.6 * rain)
    return float(np.clip(q, 0.10, 1.0))

def ekf_predict(xhat, P):
    xhat = F @ xhat
    P = F @ P @ F.T + Q
    return xhat, P

def ekf_update(xhat, P, z, H, R):
    y = np.array([[z]]) - H @ xhat
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    xhat = xhat + K @ y
    I2 = np.eye(2)
    P = (I2 - K @ H) @ P @ (I2 - K @ H).T + K @ R @ K.T
    nis = float(y.T @ np.linalg.inv(S) @ y)
    return xhat, P, nis

x_true = np.zeros((2, T))
x_true[:, 0] = np.array([0.0, 0.35])
for k in range(1, T):
    w = rng.multivariate_normal(np.zeros(2), Q)
    x_true[:, k] = F @ x_true[:, k - 1] + w

x_conf = np.array([[0.0], [0.0]])
P_conf = np.diag([1.0, 0.5])

x_fixed = np.array([[0.0], [0.0]])
P_fixed = np.diag([1.0, 0.5])

est_conf = []
est_fixed = []
nis_cam_hist = []
nis_lidar_hist = []

for k in range(T):
    fog, lux, rain = weather_profile(k)
    qc = camera_confidence(fog, lux, rain)
    ql = lidar_confidence(fog, rain)

    cam_sigma = float(np.sqrt(R_cam0[0, 0] / qc))
    lidar_sigma = float(np.sqrt(R_lidar0[0, 0] / ql))
    z_cam = x_true[0, k] + rng.normal(0.0, cam_sigma)
    z_lidar = x_true[0, k] + rng.normal(0.0, lidar_sigma)

    x_conf, P_conf = ekf_predict(x_conf, P_conf)
    x_fixed, P_fixed = ekf_predict(x_fixed, P_fixed)

    x_conf, P_conf, nis_cam = ekf_update(x_conf, P_conf, z_cam, H_cam, R_cam0 / qc)
    x_conf, P_conf, nis_lidar = ekf_update(x_conf, P_conf, z_lidar, H_lidar, R_lidar0 / ql)

    x_fixed, P_fixed, _ = ekf_update(x_fixed, P_fixed, z_cam, H_cam, R_cam0)
    x_fixed, P_fixed, _ = ekf_update(x_fixed, P_fixed, z_lidar, H_lidar, R_lidar0)

    est_conf.append(x_conf[:, 0].copy())
    est_fixed.append(x_fixed[:, 0].copy())
    nis_cam_hist.append(nis_cam)
    nis_lidar_hist.append(nis_lidar)

est_conf = np.array(est_conf)
est_fixed = np.array(est_fixed)

pos_rmse_conf = float(np.sqrt(np.mean((est_conf[:, 0] - x_true[0, :])**2)))
pos_rmse_fixed = float(np.sqrt(np.mean((est_fixed[:, 0] - x_true[0, :])**2)))

print("\n=== EKF Localization Error ===")
print(f"Position RMSE (confidence-aware): {pos_rmse_conf:.4f}")
print(f"Position RMSE (fixed covariance):  {pos_rmse_fixed:.4f}")
print(f"Mean camera NIS: {np.mean(nis_cam_hist):.3f}")
print(f"Mean LiDAR NIS:  {np.mean(nis_lidar_hist):.3f}")

k_axis = np.arange(T)

plt.figure(figsize=(10, 4))
plt.plot(x, J, label="Clear signal")
plt.plot(x, I_low, label="Fog + low-light + noise")
plt.plot(x, I_enh, label="Enhanced")
plt.xlabel("Normalized pixel coordinate")
plt.ylabel("Intensity (normalized)")
plt.title("Synthetic camera degradation and enhancement proxy")
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 4))
plt.plot(k_axis, x_true[0, :], label="True position")
plt.plot(k_axis, est_conf[:, 0], label="EKF (confidence-aware)")
plt.plot(k_axis, est_fixed[:, 0], label="EKF (fixed R)")
plt.xlabel("Time step")
plt.ylabel("Position")
plt.title("Weather-aware covariance adaptation improves robustness")
plt.legend()
plt.tight_layout()
plt.show()
