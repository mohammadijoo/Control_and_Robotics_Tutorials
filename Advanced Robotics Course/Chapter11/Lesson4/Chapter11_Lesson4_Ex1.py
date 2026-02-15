import numpy as np
from numpy.linalg import lstsq
from sklearn.cluster import KMeans

def segment_cost_gaussian(traj, lam=1e-6):
    """
    traj: array of shape (L, d) with features phi(x_t, u_t) for a candidate segment.
    Fits a Gaussian with segment-specific mean and shared spherical covariance.
    Returns negative log-likelihood up to a constant.
    """
    L, d = traj.shape
    mu = traj.mean(axis=0)
    resid = traj - mu
    var = (resid ** 2).sum() / (L * d) + lam
    # Negative log-likelihood up to constants
    return 0.5 * L * d * np.log(var)

def build_segment_cost_matrix(phi, max_len=None):
    """
    phi: array of shape (T, d).
    Returns C[i, j] = segment_cost_gaussian(phi[i:j+1]) for all i <= j.
    Optionally restrict segment length by max_len for efficiency.
    """
    T, d = phi.shape
    C = np.full((T, T), np.inf)
    for i in range(T):
        max_j = T if max_len is None else min(T, i + max_len)
        for j in range(i, max_j):
            C[i, j] = segment_cost_gaussian(phi[i:j+1])
    return C

def segment_dp(phi, beta, max_len=None):
    """
    Dynamic-programming segmentation on features phi.
    beta: penalty per segment.
    Returns list of (start, end) indices for each segment.
    """
    T = phi.shape[0]
    C = build_segment_cost_matrix(phi, max_len=max_len)
    D = np.full(T + 1, np.inf)
    prev = np.full(T + 1, -1, dtype=int)
    D[0] = 0.0

    for j in range(1, T + 1):
        best_val = np.inf
        best_i = -1
        for i in range(0, j):
            cost = D[i] + C[i, j - 1] + beta
            if cost < best_val:
                best_val = cost
                best_i = i
        D[j] = best_val
        prev[j] = best_i

    # Backtrack
    segments = []
    j = T
    while j > 0:
        i = prev[j]
        segments.append((i, j - 1))
        j = i
    segments.reverse()
    return segments

def build_skill_library(phi, segments, num_skills):
    """
    phi: (T, d) feature matrix.
    segments: list of (i, j) index pairs.
    num_skills: desired library size M.
    Returns cluster assignments and cluster centers.
    """
    seg_feats = []
    for (i, j) in segments:
        # Simple feature: mean of features over the segment
        seg_feats.append(phi[i:j+1].mean(axis=0))
    seg_feats = np.vstack(seg_feats)

    km = KMeans(n_clusters=num_skills, n_init=10)
    z = km.fit_predict(seg_feats)
    centers = km.cluster_centers_
    return z, centers

# Example usage: phi_t is concatenated joint angle and velocity over time
T = 200
d = 7 * 2
phi = np.random.randn(T, d)
segments = segment_dp(phi, beta=5.0, max_len=40)
z, centers = build_skill_library(phi, segments, num_skills=4)
print("Segments:", segments)
print("Skill assignments:", z)
      
