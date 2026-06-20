"""
Chapter 11 - Lesson 5: Lab: EKF-SLAM in a Small World
Autonomous Mobile Robots (Control Engineering)

This script:
1) Simulates a 2D "small world" with a few known-ID point landmarks.
2) Generates noisy odometry controls (v, w) and noisy range-bearing measurements.
3) Runs EKF-SLAM (joint-state EKF) with landmark initialization and gating.
4) Plots ground-truth vs estimate and prints RMSE metrics.

Dependencies: numpy, matplotlib. Optional: scipy (for chi2.ppf).
"""

import math
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Small helpers
# -----------------------------
def wrap_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def chi2_gate_2d(prob: float = 0.99) -> float:
    """Return chi-square threshold for 2 DOF at given probability."""
    try:
        from scipy.stats import chi2
        return float(chi2.ppf(prob, df=2))
    except Exception:
        # Common 2-DOF thresholds: 0.95->5.991, 0.99->9.210
        return 9.210 if prob >= 0.99 else 5.991

# -----------------------------
# Models
# -----------------------------
def motion_model(x, u, dt):
    """Unicycle motion model: x=[px,py,theta], u=[v,w]."""
    px, py, th = x
    v, w = u
    px2 = px + v * dt * math.cos(th)
    py2 = py + v * dt * math.sin(th)
    th2 = wrap_pi(th + w * dt)
    return np.array([px2, py2, th2], dtype=float)

def motion_jacobian_F(x, u, dt):
    """Jacobian of f wrt robot state x (3x3)."""
    _, _, th = x
    v, _ = u
    F = np.eye(3, dtype=float)
    F[0, 2] = -v * dt * math.sin(th)
    F[1, 2] =  v * dt * math.cos(th)
    return F

def motion_jacobian_L(x, u, dt):
    """
    Jacobian of f wrt control noise eps=[eps_v, eps_w].
    We treat noisy controls: v+eps_v, w+eps_w (2x1).
    """
    _, _, th = x
    L = np.zeros((3, 2), dtype=float)
    L[0, 0] = dt * math.cos(th)
    L[1, 0] = dt * math.sin(th)
    L[2, 1] = dt
    return L

def meas_model(xr, lm):
    """
    Range-bearing to landmark.
    xr=[px,py,theta], lm=[lx,ly].
    Returns z=[r, b]. Bearing is relative to robot heading.
    """
    px, py, th = xr
    lx, ly = lm
    dx = lx - px
    dy = ly - py
    r = math.sqrt(dx*dx + dy*dy)
    b = wrap_pi(math.atan2(dy, dx) - th)
    return np.array([r, b], dtype=float)

def meas_jacobian_H(x, lm, lm_index, n_landmarks):
    """
    Jacobian H of h wrt full state [xr; m1; ...; mN].
    lm_index: 0-based landmark index.
    """
    px, py, th = x[0], x[1], x[2]
    lx, ly = lm[0], lm[1]
    dx = lx - px
    dy = ly - py
    q = dx*dx + dy*dy
    r = math.sqrt(q) if q > 1e-12 else 1e-6

    # dh/dxr (2x3)
    Hxr = np.zeros((2, 3), dtype=float)
    Hxr[0, 0] = -dx / r
    Hxr[0, 1] = -dy / r
    Hxr[0, 2] = 0.0
    Hxr[1, 0] =  dy / q
    Hxr[1, 1] = -dx / q
    Hxr[1, 2] = -1.0

    # dh/dlm (2x2)
    Hlm = np.zeros((2, 2), dtype=float)
    Hlm[0, 0] =  dx / r
    Hlm[0, 1] =  dy / r
    Hlm[1, 0] = -dy / q
    Hlm[1, 1] =  dx / q

    dim = 3 + 2 * n_landmarks
    H = np.zeros((2, dim), dtype=float)
    H[:, 0:3] = Hxr
    j = 3 + 2 * lm_index
    H[:, j:j+2] = Hlm
    return H

def inv_meas_init(xr, z):
    """Initialize landmark from robot pose and measurement z=[r,b]."""
    px, py, th = xr
    r, b = z
    lx = px + r * math.cos(th + b)
    ly = py + r * math.sin(th + b)
    return np.array([lx, ly], dtype=float)

def inv_meas_jacobians(xr, z):
    """Jacobians of inverse measurement g(xr,z) -> landmark wrt xr (2x3) and z (2x2)."""
    px, py, th = xr
    r, b = z
    c = math.cos(th + b)
    s = math.sin(th + b)

    Gx = np.zeros((2, 3), dtype=float)
    Gx[0, 0] = 1.0
    Gx[0, 1] = 0.0
    Gx[0, 2] = -r * s
    Gx[1, 0] = 0.0
    Gx[1, 1] = 1.0
    Gx[1, 2] =  r * c

    Gz = np.zeros((2, 2), dtype=float)
    Gz[0, 0] = c
    Gz[0, 1] = -r * s
    Gz[1, 0] = s
    Gz[1, 1] =  r * c
    return Gx, Gz

# -----------------------------
# EKF-SLAM (known landmark IDs)
# -----------------------------
class EKFSLAM:
    def __init__(self, Q_control, R_meas):
        """
        Q_control: 2x2 covariance for control noise [eps_v, eps_w].
        R_meas: 2x2 covariance for measurement noise [range, bearing].
        """
        self.Q = np.array(Q_control, dtype=float)
        self.R = np.array(R_meas, dtype=float)

        self.mu = np.zeros((3,), dtype=float)          # start with robot only
        self.P = np.eye(3, dtype=float) * 1e-6
        self.nL = 0
        self.seen = {}  # id -> index in [0..nL-1]

        self.gate = chi2_gate_2d(0.99)

    def predict(self, u, dt):
        # robot part
        xr = self.mu[0:3].copy()
        xr2 = motion_model(xr, u, dt)
        F = motion_jacobian_F(xr, u, dt)
        L = motion_jacobian_L(xr, u, dt)
        Qx = L @ self.Q @ L.T

        # build big F for joint state: block-diag([F, I])
        dim = 3 + 2 * self.nL
        Fbig = np.eye(dim, dtype=float)
        Fbig[0:3, 0:3] = F

        self.mu[0:3] = xr2
        self.P = Fbig @ self.P @ Fbig.T
        self.P[0:3, 0:3] += Qx

    def _augment_with_landmark(self, lm_id, z):
        xr = self.mu[0:3].copy()
        lm = inv_meas_init(xr, z)

        # augment state
        self.mu = np.concatenate([self.mu, lm], axis=0)
        oldP = self.P
        old_dim = oldP.shape[0]

        self.nL += 1
        self.seen[lm_id] = self.nL - 1

        # Jacobians for covariance augmentation
        Gx, Gz = inv_meas_jacobians(xr, z)
        Prr = oldP[0:3, 0:3]

        Pmm = Gx @ Prr @ Gx.T + Gz @ self.R @ Gz.T

        # Cross-covariances between new landmark and existing state
        # P_xm = P_xr * Gx^T
        Px_r = oldP[:, 0:3]                 # old_dim x 3
        Pxm = Px_r @ Gx.T                   # old_dim x 2
        Pmx = Pxm.T                         # 2 x old_dim

        # assemble new P
        new_dim = old_dim + 2
        Pnew = np.zeros((new_dim, new_dim), dtype=float)
        Pnew[0:old_dim, 0:old_dim] = oldP
        Pnew[0:old_dim, old_dim:new_dim] = Pxm
        Pnew[old_dim:new_dim, 0:old_dim] = Pmx
        Pnew[old_dim:new_dim, old_dim:new_dim] = Pmm
        self.P = Pnew

    def update_one(self, lm_id, z):
        # initialize if unseen
        if lm_id not in self.seen:
            self._augment_with_landmark(lm_id, z)
            return

        idx = self.seen[lm_id]
        lm = self.mu[3 + 2*idx : 3 + 2*idx + 2]
        xr = self.mu[0:3].copy()

        zhat = meas_model(xr, lm)
        y = np.array([z[0] - zhat[0], wrap_pi(z[1] - zhat[1])], dtype=float)

        H = meas_jacobian_H(self.mu, lm, idx, self.nL)
        S = H @ self.P @ H.T + self.R

        # gating (optional)
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        d2 = float(y.T @ Sinv @ y)
        if d2 > self.gate:
            return

        K = self.P @ H.T @ Sinv
        I = np.eye(self.P.shape[0], dtype=float)

        # Joseph-stabilized update
        self.mu = self.mu + K @ y
        self.mu[2] = wrap_pi(self.mu[2])
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self.R @ K.T

        # keep symmetry numerically
        self.P = 0.5 * (self.P + self.P.T)

# -----------------------------
# Simulation
# -----------------------------
def simulate_small_world(seed=2):
    np.random.seed(seed)

    # Landmarks (ID -> position)
    landmarks = {
        1: np.array([ 4.0,  3.5]),
        2: np.array([ 8.5,  1.5]),
        3: np.array([ 7.5,  7.5]),
        4: np.array([ 2.0,  8.5]),
        5: np.array([10.0,  5.0]),
    }

    dt = 0.1
    T = 240  # steps
    # control sequence (v, w)
    U = []
    for k in range(T):
        v = 0.7 + 0.15 * math.sin(0.08 * k)
        w = 0.35 * math.sin(0.05 * k)
        U.append(np.array([v, w], dtype=float))
    U = np.array(U)

    # noise settings
    sig_v = 0.05
    sig_w = 0.03
    Q = np.diag([sig_v**2, sig_w**2])

    sig_r = 0.12
    sig_b = math.radians(2.5)
    R = np.diag([sig_r**2, sig_b**2])

    # ground truth and noisy odometry
    x = np.array([1.0, 1.0, math.radians(20.0)], dtype=float)
    Xgt = [x.copy()]
    U_noisy = []

    # measurements: list per step: [(id, z), ...]
    Z = []

    max_range = 6.0
    fov = math.radians(140.0)  # +/-70 deg

    for k in range(T):
        u = U[k].copy()
        # noisy controls used by filter
        u_noisy = np.array([u[0] + np.random.normal(0, sig_v),
                            u[1] + np.random.normal(0, sig_w)], dtype=float)
        U_noisy.append(u_noisy)

        # propagate truth with true u (for cleaner experiment); add small process disturbance
        x = motion_model(x, u, dt)
        x[0] += np.random.normal(0, 0.01)
        x[1] += np.random.normal(0, 0.01)
        x[2] = wrap_pi(x[2] + np.random.normal(0, math.radians(0.4)))
        Xgt.append(x.copy())

        obs = []
        for lm_id, lm in landmarks.items():
            ztrue = meas_model(x, lm)
            if ztrue[0] <= max_range and abs(ztrue[1]) <= fov/2.0:
                z = np.array([ztrue[0] + np.random.normal(0, sig_r),
                              wrap_pi(ztrue[1] + np.random.normal(0, sig_b))], dtype=float)
                obs.append((lm_id, z))
        Z.append(obs)

    return landmarks, dt, U_noisy, np.array(Xgt), Z, Q, R

def run():
    landmarks, dt, U_noisy, Xgt, Z, Q, R = simulate_small_world()

    ekf = EKFSLAM(Q_control=Q, R_meas=R)
    # mild initial uncertainty on pose
    ekf.P = np.diag([0.05**2, 0.05**2, math.radians(5.0)**2])

    Xest = [ekf.mu[0:3].copy()]

    for k in range(len(U_noisy)):
        ekf.predict(U_noisy[k], dt)
        for (lm_id, z) in Z[k]:
            ekf.update_one(lm_id, z)
        Xest.append(ekf.mu[0:3].copy())

    Xest = np.array(Xest)
    err = Xest[:, 0:2] - Xgt[:, 0:2]
    rmse_xy = float(np.sqrt(np.mean(np.sum(err**2, axis=1))))
    rmse_th = float(np.sqrt(np.mean((np.vectorize(wrap_pi)(Xest[:,2]-Xgt[:,2]))**2)))

    print("Final pose est:", ekf.mu[0:3])
    print("Final pose gt :", Xgt[-1])
    print("RMSE position (m):", rmse_xy)
    print("RMSE heading (rad):", rmse_th)

    # plot
    plt.figure()
    plt.plot(Xgt[:,0], Xgt[:,1], label="Ground truth")
    plt.plot(Xest[:,0], Xest[:,1], label="EKF-SLAM est")

    # landmarks: truth
    for lm_id, lm in landmarks.items():
        plt.scatter([lm[0]], [lm[1]], marker="x")
        plt.text(lm[0]+0.1, lm[1]+0.1, f"L{lm_id}")

    # landmarks: estimate
    for lm_id, idx in ekf.seen.items():
        lm_est = ekf.mu[3 + 2*idx : 3 + 2*idx + 2]
        plt.scatter([lm_est[0]], [lm_est[1]], marker="o", facecolors="none")
        plt.text(lm_est[0]+0.1, lm_est[1]-0.15, f"e{lm_id}")

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title("EKF-SLAM in a Small World")
    plt.show()

if __name__ == "__main__":
    run()
