# Chapter1_Lesson5.py
"""
Autonomous Mobile Robots — Chapter 1, Lesson 5
Typical AMR Failure Modes (drift, slip, occlusion)

This script simulates a planar mobile robot (unicycle-like) and demonstrates:
  - Drift from encoder biases / scale errors (dead-reckoning divergence)
  - Slip as unmodeled longitudinal/yaw rate attenuation and stochastic events
  - Occlusion as intermittent landmark measurement dropout and outliers
  - A lightweight linearized correction step (Gauss–Newton / one-step EKF-style)
    with residual gating for outlier rejection.

Dependencies:
  - numpy, matplotlib (standard scientific stack)
Optional:
  - pandas (only for CSV export; not required)
"""

import numpy as np
import matplotlib.pyplot as plt

def wrap_to_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi

def unicycle_step(x, u, dt):
    """
    x = [px, py, theta]
    u = [v, w]
    """
    px, py, th = x
    v, w = u
    px2 = px + dt * v * np.cos(th)
    py2 = py + dt * v * np.sin(th)
    th2 = wrap_to_pi(th + dt * w)
    return np.array([px2, py2, th2])

def landmark_measurement(x, landmark):
    """Return z=[range, bearing] from pose x to a known landmark."""
    px, py, th = x
    lx, ly = landmark
    dx, dy = lx - px, ly - py
    rng = np.sqrt(dx*dx + dy*dy)
    bear = wrap_to_pi(np.arctan2(dy, dx) - th)
    return np.array([rng, bear])

def jacobian_landmark(x, landmark):
    """
    Jacobian H = d h(x)/d x for h=[range, bearing].
    """
    px, py, th = x
    lx, ly = landmark
    dx, dy = lx - px, ly - py
    q = dx*dx + dy*dy
    rng = np.sqrt(q)
    # Avoid singularity exactly at landmark
    eps = 1e-12
    q = max(q, eps)
    rng = max(rng, eps)

    dr_dpx = -dx / rng
    dr_dpy = -dy / rng
    dr_dth = 0.0

    db_dpx =  dy / q
    db_dpy = -dx / q
    db_dth = -1.0

    H = np.array([[dr_dpx, dr_dpy, dr_dth],
                  [db_dpx, db_dpy, db_dth]])
    return H

def simulate(
    T=40.0,
    dt=0.02,
    seed=7,
    landmark=(8.0, 6.0),
    meas_period=0.5,
    p_occlude=0.35,
    p_outlier=0.08
):
    """
    Simulate true motion, encoder-based odometry (biased), and intermittent
    landmark measurements with occlusion/outliers.
    """
    rng = np.random.default_rng(seed)
    N = int(T/dt)

    # True state and estimate
    x_true = np.array([0.0, 0.0, 0.0])
    x_hat  = np.array([0.0, 0.0, 0.0])

    # Control (commanded)
    # piecewise: gentle curve with occasional turns
    def u_cmd(t):
        v = 0.6 + 0.15*np.sin(0.4*t)
        w = 0.25*np.sin(0.25*t) + 0.20*np.sin(0.05*t)
        return np.array([v, w])

    # Drift parameters (encoder biases and scale)
    b_v = 0.03          # m/s bias
    b_w = -0.015        # rad/s bias
    s_v = 1.03          # scale error (3%)
    s_w = 0.98          # scale error (-2%)

    # Slip model: attenuation + intermittent slip bursts
    # longitudinal slip ratio s_long in [0,1): v_true = (1 - s_long)*v_cmd
    # yaw slip factor s_yaw in [0,1): w_true = (1 - s_yaw)*w_cmd
    s_long_nom = 0.05
    s_yaw_nom  = 0.03
    slip_burst_prob = 0.02  # per step
    slip_burst_mag  = 0.35  # additional slip during burst

    # Noise levels
    odo_sigma_v = 0.02
    odo_sigma_w = 0.02
    meas_sigma_r = 0.08
    meas_sigma_b = np.deg2rad(1.5)

    # Correction tuning (measurement "covariance")
    R = np.diag([meas_sigma_r**2, meas_sigma_b**2])
    # Small process proxy to keep S well-conditioned
    P = np.diag([0.2**2, 0.2**2, np.deg2rad(5.0)**2])

    # Chi-square-ish gate threshold for 2D residual
    # (Not exact here; chosen as a practical numeric gate.)
    gate = 9.21  # approx 99% for chi^2 with 2 dof

    # Logs
    t_log = np.zeros(N)
    X_true = np.zeros((N,3))
    X_hat  = np.zeros((N,3))
    Z_log  = np.full((N,2), np.nan)
    Z_used = np.zeros(N, dtype=bool)

    next_meas_t = 0.0

    for k in range(N):
        t = k*dt
        u = u_cmd(t)

        # Slip realization
        burst = rng.uniform() < slip_burst_prob
        s_long = np.clip(s_long_nom + (slip_burst_mag if burst else 0.0), 0.0, 0.9)
        s_yaw  = np.clip(s_yaw_nom  + (0.5*slip_burst_mag if burst else 0.0), 0.0, 0.9)

        # True applied motion (slip affects true body velocity vs command)
        u_true = np.array([(1.0 - s_long)*u[0], (1.0 - s_yaw)*u[1]])
        x_true = unicycle_step(x_true, u_true, dt)

        # Odometry measurement (biased + noisy, ignores slip)
        u_odo = np.array([s_v*u[0] + b_v + rng.normal(0.0, odo_sigma_v),
                          s_w*u[1] + b_w + rng.normal(0.0, odo_sigma_w)])

        # Dead reckoning propagation
        x_hat = unicycle_step(x_hat, u_odo, dt)

        # Occasional landmark measurement (with occlusion/outliers)
        if t + 1e-12 >= next_meas_t:
            next_meas_t += meas_period

            occluded = rng.uniform() < p_occlude
            if not occluded:
                z = landmark_measurement(x_true, landmark).copy()

                # add noise
                z[0] += rng.normal(0.0, meas_sigma_r)
                z[1] = wrap_to_pi(z[1] + rng.normal(0.0, meas_sigma_b))

                # occasional outlier (e.g., spurious feature association)
                if rng.uniform() < p_outlier:
                    z[0] += rng.normal(0.0, 2.0)                 # large range error
                    z[1] = wrap_to_pi(z[1] + rng.normal(0.0, np.deg2rad(25.0)))

                # Log raw measurement
                Z_log[k,:] = z

                # Linearized correction at x_hat
                z_hat = landmark_measurement(x_hat, landmark)
                r = np.array([z[0] - z_hat[0], wrap_to_pi(z[1] - z_hat[1])])

                H = jacobian_landmark(x_hat, landmark)
                S = H @ P @ H.T + R
                # gating by Mahalanobis distance
                d2 = float(r.T @ np.linalg.inv(S) @ r)

                if d2 < gate:
                    # gain (like EKF gain)
                    K = P @ H.T @ np.linalg.inv(S)
                    dx = K @ r
                    x_hat = x_hat + dx
                    x_hat[2] = wrap_to_pi(x_hat[2])
                    # crude covariance update
                    P = (np.eye(3) - K @ H) @ P @ (np.eye(3) - K @ H).T + K @ R @ K.T
                    Z_used[k] = True
                else:
                    # reject measurement as inconsistent (possible occlusion/outlier)
                    Z_used[k] = False

        # Log
        t_log[k] = t
        X_true[k,:] = x_true
        X_hat[k,:]  = x_hat

        # mild covariance inflation to reflect propagation
        P = P + np.diag([1e-4, 1e-4, 1e-6])

    return t_log, X_true, X_hat, Z_log, Z_used

def main():
    t, X_true, X_hat, Z, Z_used = simulate()

    # Errors
    e = X_hat - X_true
    e[:,2] = np.vectorize(wrap_to_pi)(e[:,2])

    plt.figure()
    plt.plot(X_true[:,0], X_true[:,1], label="true")
    plt.plot(X_hat[:,0],  X_hat[:,1],  label="estimated")
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory: true vs estimated (drift + slip + occlusion)")
    plt.legend()

    plt.figure()
    plt.plot(t, e[:,0], label="ex")
    plt.plot(t, e[:,1], label="ey")
    plt.plot(t, e[:,2], label="etheta")
    plt.xlabel("time [s]")
    plt.ylabel("error")
    plt.title("Estimation error")
    plt.legend()

    # Measurement usage markers
    idx = np.where(~np.isnan(Z[:,0]))[0]
    if len(idx) > 0:
        plt.figure()
        plt.plot(t[idx], Z[idx,0], ".", label="range measurements")
        plt.plot(t[idx][Z_used[idx]], Z[idx,0][Z_used[idx]], "o", fillstyle="none",
                 label="used after gating")
        plt.xlabel("time [s]")
        plt.ylabel("range [m]")
        plt.title("Landmark measurement stream (occlusion/outliers rejected)")
        plt.legend()

    plt.show()

if __name__ == "__main__":
    main()
