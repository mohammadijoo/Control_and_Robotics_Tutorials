"""
Chapter1_Lesson4.py
Sensing–Estimation–Navigation pipeline demo (2D unicycle) using weighted least squares (WLS)
prior + range-bearing landmark measurements.
Dependencies: numpy, matplotlib
Optional robotics ecosystem pointers: ROS 2 (rclpy), nav2, GTSAM (not required for this demo).
"""

import numpy as np
import matplotlib.pyplot as plt

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def unicycle_step(x: np.ndarray, v: float, w: float, dt: float) -> np.ndarray:
    """Discrete-time unicycle integration."""
    x_new = x.copy()
    th = x[2]
    x_new[0] = x[0] + v * dt * np.cos(th)
    x_new[1] = x[1] + v * dt * np.sin(th)
    x_new[2] = wrap_angle(x[2] + w * dt)
    return x_new

def meas_model(x: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    """Range-bearing measurement z = [r, b] to landmark in world coordinates."""
    dx = landmark[0] - x[0]
    dy = landmark[1] - x[1]
    r = np.hypot(dx, dy)
    b = wrap_angle(np.arctan2(dy, dx) - x[2])
    return np.array([r, b])

def meas_jacobian(x: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    """Jacobian H = d h / d x for range-bearing measurement."""
    dx = landmark[0] - x[0]
    dy = landmark[1] - x[1]
    q = dx*dx + dy*dy
    r = np.sqrt(q)

    # Avoid division by zero
    eps = 1e-9
    q = max(q, eps)
    r = max(r, eps)

    # h1 = r = sqrt((lx-x)^2 + (ly-y)^2)
    dr_dx = -(dx) / r
    dr_dy = -(dy) / r
    dr_dth = 0.0

    # h2 = b = atan2(dy, dx) - th
    db_dx =  dy / q
    db_dy = -dx / q
    db_dth = -1.0

    return np.array([[dr_dx, dr_dy, dr_dth],
                     [db_dx, db_dy, db_dth]])

def wls_update_iterated(x_prior: np.ndarray, P_prior: np.ndarray,
                        z_list: list, landmarks: np.ndarray,
                        R: np.ndarray, iters: int = 2) -> tuple[np.ndarray, np.ndarray]:
    """
    Iterated WLS (Gauss-Newton) for a prior + stacked measurements.
    Minimizes:
        (x-x_prior)^T P_prior^{-1} (x-x_prior) + sum_i (z_i - h_i(x))^T R^{-1} (z_i - h_i(x))
    """
    if len(z_list) == 0:
        return x_prior, P_prior

    # Stack measurements
    z = np.concatenate(z_list, axis=0)  # shape (2m,)
    m = len(z_list)

    # Block-diagonal measurement covariance
    R_big = np.kron(np.eye(m), R)  # (2m x 2m)
    Rinv = np.linalg.inv(R_big)

    Pinv = np.linalg.inv(P_prior)

    x = x_prior.copy()
    for _ in range(iters):
        # Build stacked residual and Jacobian
        h = []
        H = []
        for i in range(m):
            hi = meas_model(x, landmarks[i])
            h.append(hi)
            H.append(meas_jacobian(x, landmarks[i]))
        h = np.concatenate(h, axis=0)
        H = np.vstack(H)

        # Residual (bearing residual must be wrapped)
        res = z - h
        for i in range(m):
            res[2*i + 1] = wrap_angle(res[2*i + 1])

        # Solve normal equations for delta:
        # (P^{-1} + H^T R^{-1} H) delta = H^T R^{-1} res + P^{-1}(x_prior - x)
        A = Pinv + H.T @ Rinv @ H
        b = H.T @ Rinv @ res + Pinv @ (x_prior - x)
        delta = np.linalg.solve(A, b)

        x = x + delta
        x[2] = wrap_angle(x[2])

    # Posterior covariance at final linearization point
    h = []
    H = []
    for i in range(m):
        h.append(meas_model(x, landmarks[i]))
        H.append(meas_jacobian(x, landmarks[i]))
    H = np.vstack(H)
    P_post = np.linalg.inv(Pinv + H.T @ Rinv @ H)
    return x, P_post

def main():
    np.random.seed(7)

    # Map (known landmarks): choose 3 well-spread landmarks
    landmarks = np.array([[5.0, 0.0],
                          [6.0, 6.0],
                          [0.0, 6.0]])

    dt = 0.1
    N = 350

    # True state and estimate (x, y, theta)
    x_true = np.array([0.0, 0.0, 0.0])
    x_hat  = np.array([0.0, 0.0, 0.0])

    # Prior covariance
    P = np.diag([0.15**2, 0.15**2, (np.deg2rad(8.0))**2])

    # Noise levels
    sigma_v = 0.08          # m/s odometry noise
    sigma_w = np.deg2rad(3) # rad/s odometry noise
    sigma_r = 0.12          # m range noise
    sigma_b = np.deg2rad(2) # rad bearing noise

    # Navigation goal and controller gains (simple go-to-goal)
    goal = np.array([7.0, 7.0])
    k_heading = 1.6
    v_max = 0.9

    # Measurement covariance per landmark measurement
    R = np.diag([sigma_r**2, sigma_b**2])

    hist_true = np.zeros((N, 3))
    hist_hat  = np.zeros((N, 3))

    for k in range(N):
        # --- Navigation (policy uses estimate) ---
        dx = goal[0] - x_hat[0]
        dy = goal[1] - x_hat[1]
        dist = np.hypot(dx, dy)
        desired_heading = np.arctan2(dy, dx)
        heading_error = wrap_angle(desired_heading - x_hat[2])

        v_cmd = v_max * np.tanh(dist)      # smoothly saturating speed
        w_cmd = k_heading * heading_error  # proportional heading control

        # --- True motion (includes unmodeled disturbances) ---
        # Model a mild lateral slip disturbance as small random heading perturbation.
        slip = np.random.normal(0.0, np.deg2rad(0.35))
        x_true = unicycle_step(x_true, v_cmd, w_cmd + slip/dt, dt)

        # --- Sensing: odometry (control-derived) ---
        v_meas = v_cmd + np.random.normal(0.0, sigma_v)
        w_meas = w_cmd + np.random.normal(0.0, sigma_w)

        # --- Estimation: prediction from odometry ---
        x_prior = unicycle_step(x_hat, v_meas, w_meas, dt)

        th = x_hat[2]
        F = np.array([[1.0, 0.0, -v_meas*dt*np.sin(th)],
                      [0.0, 1.0,  v_meas*dt*np.cos(th)],
                      [0.0, 0.0,  1.0]])

        # Simple process noise model (odometry-induced)
        Q = np.diag([(sigma_v*dt)**2, (sigma_v*dt)**2, (sigma_w*dt)**2])
        P_prior = F @ P @ F.T + Q

        # --- Sensing: landmark measurements (range-bearing) ---
        z_list = []
        # Use all landmarks within range to emulate a perception module
        max_range = 8.0
        for lm in landmarks:
            z_true = meas_model(x_true, lm)
            if z_true[0] <= max_range:
                z_noisy = z_true + np.array([np.random.normal(0.0, sigma_r),
                                             np.random.normal(0.0, sigma_b)])
                z_noisy[1] = wrap_angle(z_noisy[1])
                z_list.append(z_noisy)

        # --- Estimation: iterated WLS correction (prior + measurements) ---
        x_hat, P = wls_update_iterated(x_prior, P_prior, z_list, landmarks[:len(z_list)], R, iters=2)

        hist_true[k] = x_true
        hist_hat[k]  = x_hat

    # --- Visualization ---
    plt.figure()
    plt.plot(hist_true[:,0], hist_true[:,1], label="true")
    plt.plot(hist_hat[:,0],  hist_hat[:,1],  label="estimated")
    plt.scatter(landmarks[:,0], landmarks[:,1], marker="x", label="landmarks")
    plt.scatter(goal[0], goal[1], marker="*", label="goal")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title("Sensing–Estimation–Navigation loop (demo)")
    plt.show()

if __name__ == "__main__":
    main()
