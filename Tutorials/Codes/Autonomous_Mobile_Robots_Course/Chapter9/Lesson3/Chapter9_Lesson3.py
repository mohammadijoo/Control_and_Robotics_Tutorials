# Chapter9_Lesson3.py
# Feature-Based Mapping (Landmarks) with a per-landmark EKF (pose assumed known).
# This script simulates a 2D robot trajectory, range-bearing observations to point landmarks,
# performs nearest-neighbor data association with Mahalanobis gating, initializes new landmarks,
# and updates landmark estimates with an EKF measurement update.

import numpy as np
import matplotlib.pyplot as plt

np.random.seed(7)

def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def h_rb(x, m):
    """Range-bearing measurement model z = [r, b] from pose x=[px,py,th] to landmark m=[mx,my]."""
    dx = m[0] - x[0]
    dy = m[1] - x[1]
    r = np.sqrt(dx*dx + dy*dy)
    b = wrap_angle(np.arctan2(dy, dx) - x[2])
    return np.array([r, b])

def H_rb_wrt_m(x, m):
    """Jacobian H = dh/dm for range-bearing wrt landmark position."""
    dx = m[0] - x[0]
    dy = m[1] - x[1]
    q = dx*dx + dy*dy
    r = np.sqrt(q)
    if r < 1e-9:
        r = 1e-9
        q = r*r
    # dr/dmx = dx/r , dr/dmy = dy/r
    # db/dmx = -dy/q , db/dmy = dx/q
    return np.array([[dx/r, dy/r],
                     [-dy/q,  dx/q]])

def inv_h_rb(x, z):
    """Inverse observation (used for initialization): landmark position from pose and measurement."""
    r, b = z
    ang = x[2] + b
    return np.array([x[0] + r*np.cos(ang), x[1] + r*np.sin(ang)])

def simulate_world():
    # True landmarks
    M_true = np.array([
        [-6.0,  5.0],
        [-2.0, -4.0],
        [ 4.0,  6.0],
        [ 7.0, -2.0],
        [ 1.0,  1.5],
        [-7.0, -6.0],
    ])
    # Robot trajectory (known poses for mapping): a lopsided loop
    T = 180
    t = np.linspace(0, 2*np.pi, T)
    px = 2.5*np.cos(t) + 0.5*np.cos(3*t)
    py = 2.0*np.sin(t)
    th = wrap_angle(np.arctan2(np.gradient(py), np.gradient(px)))
    X = np.vstack([px, py, th]).T

    # Sensor parameters
    r_max = 9.0
    sigma_r = 0.15
    sigma_b = np.deg2rad(2.0)
    R = np.diag([sigma_r**2, sigma_b**2])

    # Generate measurements (with unknown associations in the algorithm)
    Z = []  # list of list of (z, true_id)
    for k in range(T):
        zk = []
        for i, mi in enumerate(M_true):
            z = h_rb(X[k], mi)
            if z[0] <= r_max:
                z_noisy = z + np.array([np.random.normal(0, sigma_r),
                                        np.random.normal(0, sigma_b)])
                z_noisy[1] = wrap_angle(z_noisy[1])
                zk.append((z_noisy, i))
        Z.append(zk)
    return X, M_true, Z, R

class LandmarkMapEKF:
    def __init__(self, R, gate_chi2=9.21):
        """
        R: measurement covariance (2x2)
        gate_chi2: chi-square gate for 2 dof (e.g., 9.21 ~ 99%).
        """
        self.R = R
        self.gate = gate_chi2
        self.mu = []      # list of 2-vectors
        self.Sigma = []   # list of 2x2 covariances
        self.active = []  # list of bool
    def num(self):
        return len(self.mu)

    def add_landmark(self, x, z, init_sigma=1.5):
        m0 = inv_h_rb(x, z)
        self.mu.append(m0)
        self.Sigma.append(np.eye(2) * (init_sigma**2))
        self.active.append(True)

    def associate(self, x, z):
        """
        Nearest-neighbor association using Mahalanobis distance in measurement space.
        Returns (best_index or None, best_d2).
        """
        if self.num() == 0:
            return None, np.inf

        best_j, best_d2 = None, np.inf
        for j in range(self.num()):
            if not self.active[j]:
                continue
            m = self.mu[j]
            H = H_rb_wrt_m(x, m)
            zhat = h_rb(x, m)
            innov = np.array([z[0] - zhat[0], wrap_angle(z[1] - zhat[1])])
            S = H @ self.Sigma[j] @ H.T + self.R
            try:
                Sinv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue
            d2 = float(innov.T @ Sinv @ innov)
            if d2 < best_d2:
                best_d2 = d2
                best_j = j

        if best_d2 <= self.gate:
            return best_j, best_d2
        return None, best_d2

    def update(self, x, z, j):
        """EKF update of landmark j with measurement z at known pose x."""
        m = self.mu[j]
        P = self.Sigma[j]

        H = H_rb_wrt_m(x, m)
        zhat = h_rb(x, m)
        y = np.array([z[0] - zhat[0], wrap_angle(z[1] - zhat[1])])

        S = H @ P @ H.T + self.R
        K = P @ H.T @ np.linalg.inv(S)

        m_new = m + K @ y
        P_new = (np.eye(2) - K @ H) @ P

        self.mu[j] = m_new
        self.Sigma[j] = 0.5*(P_new + P_new.T)  # symmetrize

def run():
    X, M_true, Z, R = simulate_world()
    mapper = LandmarkMapEKF(R=R, gate_chi2=9.21)

    # Process measurements sequentially
    for k, x in enumerate(X):
        for (z, _true_id) in Z[k]:
            j, d2 = mapper.associate(x, z)
            if j is None:
                mapper.add_landmark(x, z)
            else:
                mapper.update(x, z, j)

    M_est = np.array(mapper.mu) if mapper.num() else np.zeros((0,2))

    # Plot
    plt.figure()
    plt.plot(X[:,0], X[:,1], '-', label="robot path (known)")
    plt.scatter(M_true[:,0], M_true[:,1], marker='x', s=80, label="true landmarks")
    if len(M_est):
        plt.scatter(M_est[:,0], M_est[:,1], marker='o', label="estimated landmarks")
        # draw covariance ellipses (2-sigma) for a subset
        for j in range(min(12, mapper.num())):
            P = mapper.Sigma[j]
            w, V = np.linalg.eigh(P)
            w = np.maximum(w, 1e-12)
            ang = np.linspace(0, 2*np.pi, 60)
            ell = (V @ (2.0*np.sqrt(w)[:,None]*np.vstack([np.cos(ang), np.sin(ang)])))
            plt.plot(mapper.mu[j][0] + ell[0,:], mapper.mu[j][1] + ell[1,:], linewidth=1)

    plt.axis('equal')
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Feature-Based Mapping with per-landmark EKF")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    run()
