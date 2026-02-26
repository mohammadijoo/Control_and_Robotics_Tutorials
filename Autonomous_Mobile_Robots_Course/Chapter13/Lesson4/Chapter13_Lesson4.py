# Chapter13_Lesson4.py
# Visual/VIO SLAM robustness: (1) monocular scale ambiguity, (2) lighting change via affine photometric model,
# (3) motion blur metric via variance of Laplacian.
#
# Dependencies: numpy, opencv-python
#
# Run:
#   python Chapter13_Lesson4.py

import numpy as np
import cv2


def project_points(K, R, t, Xw):
    '''
    Pinhole projection with intrinsics K:
      u = pi( K [R|t] X )
    '''
    Xc = (R @ Xw.T + t.reshape(3, 1)).T  # Nx3
    x = Xc[:, :2] / Xc[:, 2:3]          # normalized
    u = (K[:2, :2] @ x.T + K[:2, 2:3]).T
    return u, Xc


def monocular_two_view_scale_demo(seed=0):
    np.random.seed(seed)

    fx = fy = 420.0
    cx, cy = 320.0, 240.0
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1.0]], dtype=float)

    # 3D points (world) in front of camera 1
    N = 200
    Xw = np.column_stack([
        np.random.uniform(-2.0, 2.0, N),
        np.random.uniform(-1.0, 1.0, N),
        np.random.uniform(4.0, 8.0, N),
    ])

    R1 = np.eye(3)
    t1 = np.zeros(3)

    # Pose 2: small yaw + forward translation (true metric, unknown to monocular)
    yaw = np.deg2rad(5.0)
    R2 = np.array([[np.cos(yaw), 0, np.sin(yaw)],
                   [0, 1, 0],
                   [-np.sin(yaw), 0, np.cos(yaw)]], dtype=float)
    t2_metric = np.array([0.20, 0.0, 0.80])  # meters

    u1, _ = project_points(K, R1, t1, Xw)
    u2, _ = project_points(K, R2, t2_metric, Xw)

    # Add pixel noise
    noise = 0.5
    u1n = u1 + np.random.normal(0, noise, u1.shape)
    u2n = u2 + np.random.normal(0, noise, u2.shape)

    E, _inl = cv2.findEssentialMat(u1n, u2n, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    _, _R_est, t_est, _mask = cv2.recoverPose(E, u1n, u2n, K)

    t_unit = t_est.reshape(3)
    t_unit_norm = np.linalg.norm(t_unit)

    print("\n--- Monocular 2-view: translation scale ambiguity ---")
    print("Recovered translation direction (unit up to noise):", t_unit)
    print("||t_unit||:", t_unit_norm)
    print("True metric translation:", t2_metric, "||t_metric||:", np.linalg.norm(t2_metric))

    # Suppose an IMU-based integration provides metric delta-position over the same interval.
    # Here we fake it as ground truth + small noise.
    delta_p_imu = t2_metric + np.random.normal(0, 0.02, 3)

    scale_hat = np.linalg.norm(delta_p_imu) / (t_unit_norm + 1e-12)
    t_metric_hat = scale_hat * t_unit

    print("IMU delta_p (fake):", delta_p_imu)
    print("Estimated scale_hat:", scale_hat)
    print("Metric translation from scale_hat * t_unit:", t_metric_hat)


def affine_brightness_fit(I1, I2, mask=None):
    '''
    Solve min_{a,b} sum_p (a I1(p) + b - I2(p))^2.
    Returns (a,b).
    '''
    if mask is None:
        mask = np.ones(I1.shape, dtype=bool)
    x = I1[mask].reshape(-1, 1).astype(np.float64)
    y = I2[mask].reshape(-1, 1).astype(np.float64)
    A = np.hstack([x, np.ones_like(x)])
    theta, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
    a, b = float(theta[0]), float(theta[1])
    return a, b


def blur_score_var_laplacian(gray):
    L = cv2.Laplacian(gray, cv2.CV_64F)
    return float(L.var())


def photometric_demo():
    print("\n--- Lighting change + blur demo (synthetic) ---")

    img = np.zeros((240, 320), dtype=np.uint8)
    cv2.putText(img, "AMR", (60, 140), cv2.FONT_HERSHEY_SIMPLEX, 2.0, 255, 3, cv2.LINE_AA)

    a_true, b_true = 1.2, -10.0
    img2 = np.clip(a_true * img.astype(np.float64) + b_true, 0, 255).astype(np.uint8)

    # Add blur (simulating motion blur / defocus)
    img2 = cv2.GaussianBlur(img2, (9, 9), 2.5)

    a_hat, b_hat = affine_brightness_fit(img, img2)
    print("True (a,b):", (a_true, b_true))
    print("Estimated (a,b):", (a_hat, b_hat))

    s1 = blur_score_var_laplacian(img)
    s2 = blur_score_var_laplacian(img2)
    print("Blur score var(Laplacian): sharp=", s1, " blurred=", s2)


def main():
    monocular_two_view_scale_demo(seed=0)
    photometric_demo()


if __name__ == "__main__":
    main()
