# Chapter13_Lesson3.py
# Visual–Inertial Fusion Pipelines (AMR Focus)
#
# Minimal, self-contained EKF-style fusion of IMU propagation + visual pose updates.
# This script simulates a planar robot with an IMU and a visual pose measurement
# (e.g., from PnP / monocular VO with external scale, stereo VO, or wheel-aided VO)
# and fuses them using an error-state EKF.
#
# Dependencies: numpy (required)
# Optional: scipy (for chi-square gating), matplotlib (for plots)

import numpy as np

try:
    from scipy.stats import chi2
except Exception:
    chi2 = None

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None


def wrap_angle(theta: float) -> float:
    'Wrap angle to (-pi, pi].'
    return (theta + np.pi) % (2.0 * np.pi) - np.pi


def rot2(theta: float) -> np.ndarray:
    '2D rotation matrix.'
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def imu_propagate(x, P, u, Qc, dt, g=np.array([0.0, -9.81])):
    '''
    State x = [px, py, vx, vy, theta, bax, bay, bg]^T
    IMU measurement u = [ax_m, ay_m, wz_m]^T (body-frame specific force and yaw rate)
    Continuous/white noise proxy packed in Qc (8x8). For didactics we use Qd = Qc * dt.

    Returns propagated (x, P).
    '''
    px, py, vx, vy, th, bax, bay, bg = x
    axm, aym, wzm = u

    a_b = np.array([axm - bax, aym - bay])
    w = wzm - bg

    th_new = wrap_angle(th + w * dt)
    R = rot2(th)
    a_w = R @ a_b + g

    vx_new = vx + a_w[0] * dt
    vy_new = vy + a_w[1] * dt
    px_new = px + vx * dt + 0.5 * a_w[0] * dt * dt
    py_new = py + vy * dt + 0.5 * a_w[1] * dt * dt

    x_new = np.array([px_new, py_new, vx_new, vy_new, th_new, bax, bay, bg], dtype=float)

    F = np.eye(8)
    F[0, 2] = dt
    F[1, 3] = dt

    dR_dth = np.array([[-np.sin(th), -np.cos(th)],
                       [ np.cos(th), -np.sin(th)]])
    da_w_dth = dR_dth @ a_b

    F[2, 4] = da_w_dth[0] * dt
    F[3, 4] = da_w_dth[1] * dt

    F[2, 5] = -R[0, 0] * dt
    F[2, 6] = -R[0, 1] * dt
    F[3, 5] = -R[1, 0] * dt
    F[3, 6] = -R[1, 1] * dt

    F[0, 4] = da_w_dth[0] * 0.5 * dt * dt
    F[1, 4] = da_w_dth[1] * 0.5 * dt * dt
    F[0, 5] = -R[0, 0] * 0.5 * dt * dt
    F[0, 6] = -R[0, 1] * 0.5 * dt * dt
    F[1, 5] = -R[1, 0] * 0.5 * dt * dt
    F[1, 6] = -R[1, 1] * 0.5 * dt * dt

    F[4, 7] = -dt

    Qd = Qc * dt
    P_new = F @ P @ F.T + Qd
    return x_new, P_new


def ekf_update_pose(x, P, z, Rz, gate_prob=0.99):
    '''Visual measurement: z = [px, py, theta] + noise.'''
    H = np.zeros((3, 8))
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    H[2, 4] = 1.0

    zhat = np.array([x[0], x[1], x[4]], dtype=float)
    y = z - zhat
    y[2] = wrap_angle(y[2])

    S = H @ P @ H.T + Rz

    if chi2 is not None:
        d2 = float(y.T @ np.linalg.inv(S) @ y)
        thr = chi2.ppf(gate_prob, df=3)
        if d2 > thr:
            return x, P, False, d2, thr

    K = P @ H.T @ np.linalg.inv(S)
    dx = K @ y
    x_upd = x + dx
    x_upd[4] = wrap_angle(x_upd[4])

    I = np.eye(len(x))
    P_upd = (I - K @ H) @ P @ (I - K @ H).T + K @ Rz @ K.T
    return x_upd, P_upd, True, None, None


def simulate_planar_motion(T=20.0, dt=0.01, vo_dt=0.1, seed=7):
    '''Return truth and sensor streams for a planar robot.'''
    rng = np.random.default_rng(seed)
    N = int(T / dt) + 1
    t = np.linspace(0.0, T, N)

    px = np.zeros(N)
    py = np.zeros(N)
    vx = np.zeros(N)
    vy = np.zeros(N)
    th = np.zeros(N)

    speed = 1.0 + 0.2 * np.sin(0.3 * t)
    yaw_rate = 0.25 * np.sin(0.2 * t)

    for k in range(1, N):
        th[k] = wrap_angle(th[k - 1] + yaw_rate[k - 1] * dt)
        vx[k] = speed[k] * np.cos(th[k])
        vy[k] = speed[k] * np.sin(th[k])
        px[k] = px[k - 1] + vx[k - 1] * dt
        py[k] = py[k - 1] + vy[k - 1] * dt

    g = np.array([0.0, -9.81])
    ax_w = np.gradient(vx, dt)
    ay_w = np.gradient(vy, dt)
    a_w = np.vstack([ax_w, ay_w]).T

    imu = np.zeros((N, 3))
    b_a = np.array([0.08, -0.05])
    b_g = 0.01
    sigma_a = 0.12
    sigma_g = 0.015

    for k in range(N):
        R = rot2(th[k])
        a_b = R.T @ (a_w[k] - g)
        imu[k, 0:2] = a_b + b_a + rng.normal(0.0, sigma_a, size=2)
        imu[k, 2] = yaw_rate[k] + b_g + rng.normal(0.0, sigma_g)

    vo_step = int(vo_dt / dt)
    vo_idx = np.arange(0, N, vo_step)
    vo_t = t[vo_idx]
    vo = np.zeros((len(vo_idx), 3))
    sigma_p = 0.05
    sigma_th = 0.02
    for i, k in enumerate(vo_idx):
        vo[i, 0] = px[k] + rng.normal(0.0, sigma_p)
        vo[i, 1] = py[k] + rng.normal(0.0, sigma_p)
        vo[i, 2] = wrap_angle(th[k] + rng.normal(0.0, sigma_th))

    x_true = np.vstack([px, py, vx, vy, th]).T
    return t, x_true, imu, vo_t, vo


def main():
    dt = 0.01
    T = 20.0
    vo_dt = 0.1

    t, x_true, imu, vo_t, vo = simulate_planar_motion(T=T, dt=dt, vo_dt=vo_dt)

    x = np.zeros(8)
    x[0:2] = vo[0, 0:2]
    x[4] = vo[0, 2]
    P = np.diag([0.5, 0.5, 0.2, 0.2, 0.3, 0.2, 0.2, 0.1]) ** 2

    Qc = np.diag([0.0] * 8)
    Qc[2, 2] = 0.4**2
    Qc[3, 3] = 0.4**2
    Qc[4, 4] = 0.15**2
    Qc[5, 5] = 0.01**2
    Qc[6, 6] = 0.01**2
    Qc[7, 7] = 0.002**2

    Rz = np.diag([0.05, 0.05, 0.02]) ** 2

    est = np.zeros((len(t), 8))
    vo_ptr = 0

    for k in range(len(t)):
        x, P = imu_propagate(x, P, imu[k], Qc, dt)
        if vo_ptr < len(vo_t) and abs(t[k] - vo_t[vo_ptr]) < 0.5 * dt:
            x, P, used, d2, thr = ekf_update_pose(x, P, vo[vo_ptr], Rz)
            vo_ptr += 1
        est[k] = x

    final_err = np.array([est[-1, 0] - x_true[-1, 0],
                          est[-1, 1] - x_true[-1, 1],
                          wrap_angle(est[-1, 4] - x_true[-1, 4])])
    print('Final error [m, m, rad]:', final_err)

    if plt is not None:
        plt.figure()
        plt.plot(x_true[:, 0], x_true[:, 1], label='truth')
        plt.plot(est[:, 0], est[:, 1], label='EKF')
        plt.axis('equal')
        plt.legend()
        plt.title('Planar VIO fusion: trajectory')
        plt.show()


if __name__ == '__main__':
    main()
