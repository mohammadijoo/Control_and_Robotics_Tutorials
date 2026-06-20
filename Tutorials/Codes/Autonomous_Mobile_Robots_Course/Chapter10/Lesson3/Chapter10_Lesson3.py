# Chapter10_Lesson3.py
"""
Correlative Scan Matching (2D) — educational reference implementation.

- Builds a 2D occupancy grid map (binary), then a likelihood field (smoothed)
  using a distance transform.
- Performs (multi-resolution) discrete search around an initial guess for
  (x, y, theta) to maximize correlation score.

Dependencies:
  - numpy
  - scipy (optional but recommended) for distance_transform_edt

This is NOT a full SLAM system; it is the scan-to-map matcher used as a
front-end measurement update in localization/SLAM pipelines.
"""

from __future__ import annotations
import math
import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
    _HAVE_SCIPY = True
except Exception:
    distance_transform_edt = None
    _HAVE_SCIPY = False


def se2_from_xytheta(x: float, y: float, theta: float) -> np.ndarray:
    """Homogeneous transform T (3x3) for SE(2)."""
    c, s = math.cos(theta), math.sin(theta)
    T = np.array([[c, -s, x],
                  [s,  c, y],
                  [0.0, 0.0, 1.0]], dtype=float)
    return T


def inv_se2(T: np.ndarray) -> np.ndarray:
    """Inverse of 3x3 SE(2) transform."""
    R = T[:2, :2]
    t = T[:2, 2]
    Rt = R.T
    Tin = np.eye(3)
    Tin[:2, :2] = Rt
    Tin[:2, 2] = -Rt @ t
    return Tin


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply SE(2) transform to Nx2 points."""
    assert pts.ndim == 2 and pts.shape[1] == 2
    R = T[:2, :2]
    t = T[:2, 2]
    return (pts @ R.T) + t


def make_synthetic_map(res: float = 0.05, size_m: float = 10.0) -> tuple[np.ndarray, dict]:
    """
    Make a simple square room with an interior pillar in a binary occupancy grid.

    Returns:
      occ (H,W) uint8 in {0,1}, meta dict with origin and resolution.
    """
    W = int(size_m / res)
    H = int(size_m / res)
    occ = np.zeros((H, W), dtype=np.uint8)

    # Walls: 1-cell thick
    occ[0, :] = 1
    occ[-1, :] = 1
    occ[:, 0] = 1
    occ[:, -1] = 1

    # Interior pillar (a small square)
    cx, cy = W // 2, H // 2
    r = int(0.4 / res)  # 40 cm half-size
    occ[cy - r:cy + r + 1, cx - r:cx + r + 1] = 1

    meta = {
        "resolution": res,
        "origin_xy": np.array([0.0, 0.0], dtype=float),  # world (0,0) at grid (0,0)
    }
    return occ, meta


def occ_points_world(occ: np.ndarray, meta: dict, stride: int = 2) -> np.ndarray:
    """Convert occupied grid cells to world points (centers), subsampled."""
    res = float(meta["resolution"])
    ys, xs = np.where(occ[::stride, ::stride] > 0)
    xs = xs * stride
    ys = ys * stride
    pts = np.stack([(xs + 0.5) * res, (ys + 0.5) * res], axis=1)
    return pts


def simulate_scan_endpoints(occ: np.ndarray, meta: dict, T_w_r_true: np.ndarray,
                            max_range: float = 6.0, noise_std: float = 0.01) -> np.ndarray:
    """
    Simulate a LiDAR scan as a set of endpoint points in the robot frame.

    We do NOT raycast. Instead, we treat all occupied map points within range as potential
    returns and sample from them. This is enough to demonstrate correlative matching.
    """
    pts_w = occ_points_world(occ, meta, stride=2)
    p_r = transform_points(inv_se2(T_w_r_true), pts_w)  # map points expressed in robot frame
    d = np.linalg.norm(p_r, axis=1)
    keep = (d > 0.3) & (d < max_range)
    p_r = p_r[keep]

    # Randomly subsample to mimic a scan
    if len(p_r) > 600:
        idx = np.random.choice(len(p_r), size=600, replace=False)
        p_r = p_r[idx]

    p_r = p_r + np.random.normal(0.0, noise_std, size=p_r.shape)
    return p_r.astype(float)


def likelihood_field_from_occ(occ: np.ndarray, sigma_m: float, res: float) -> np.ndarray:
    """
    Convert binary occupancy to a likelihood field:
      field(x) = exp( -d(x)^2 / (2*sigma^2) ),
    where d(x) is distance to nearest occupied cell.
    """
    if not _HAVE_SCIPY:
        # Fallback: crude approximation by repeated dilation distance (Manhattan)
        # NOTE: for best results install SciPy.
        dist = np.full_like(occ, fill_value=1e9, dtype=float)
        dist[occ > 0] = 0.0
        # 4-neighbor wavefront (few iterations)
        for _ in range(200):
            dist = np.minimum(dist, np.roll(dist, 1, axis=0) + 1)
            dist = np.minimum(dist, np.roll(dist, -1, axis=0) + 1)
            dist = np.minimum(dist, np.roll(dist, 1, axis=1) + 1)
            dist = np.minimum(dist, np.roll(dist, -1, axis=1) + 1)
        dist_m = dist * res
    else:
        # distance to nearest occupied: distance transform on free-space mask
        free = (occ == 0)
        dist_m = distance_transform_edt(free) * res

    sigma = float(sigma_m)
    field = np.exp(-(dist_m ** 2) / (2.0 * sigma ** 2))
    return field.astype(np.float32)


def downsample_max(grid: np.ndarray) -> np.ndarray:
    """2x downsample using max-pooling (preserves peaks for correlation)."""
    H, W = grid.shape
    H2, W2 = H // 2, W // 2
    g = grid[:2*H2, :2*W2]
    g = g.reshape(H2, 2, W2, 2).max(axis=(1, 3))
    return g


def build_pyramid(field: np.ndarray, levels: int) -> list[np.ndarray]:
    pyr = [field]
    for _ in range(1, levels):
        pyr.append(downsample_max(pyr[-1]))
    return pyr


def score_pose(field: np.ndarray, meta: dict, pts_r: np.ndarray,
               x: float, y: float, theta: float, res_level: float) -> float:
    """Compute correlation score for a candidate pose."""
    T_w_r = se2_from_xytheta(x, y, theta)
    pts_w = transform_points(T_w_r, pts_r)

    # world -> grid index
    res = res_level
    gx = np.floor(pts_w[:, 0] / res).astype(int)
    gy = np.floor(pts_w[:, 1] / res).astype(int)

    H, W = field.shape
    inside = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
    if not np.any(inside):
        return -1e18
    return float(np.sum(field[gy[inside], gx[inside]]))


def multires_correlative_match(field_pyr: list[np.ndarray], meta: dict, pts_r: np.ndarray,
                              x0: float, y0: float, th0: float,
                              search_xy_m: float = 0.5, search_th_rad: float = math.radians(10),
                              step_xy_m: float = 0.05, step_th_rad: float = math.radians(1.0),
                              top_k: int = 40) -> tuple[float, float, float, float]:
    """
    Coarse-to-fine search. At each pyramid level (coarsest -> finest),
    evaluate a discrete grid of candidates around current best, keep top-K, refine.

    Returns: (x, y, theta, score)
    """
    levels = len(field_pyr)
    # pyramid meta: resolution doubles each level upward
    base_res = float(meta["resolution"])
    candidates = [(x0, y0, th0, 0.0)]

    for li in reversed(range(levels)):  # coarsest first
        field = field_pyr[li]
        res_level = base_res * (2 ** li)

        # scale step with level (coarser => bigger step in meters)
        step_xy = max(step_xy_m, res_level)
        step_th = step_th_rad * (2 ** li)  # coarser uses wider angular steps

        new_cands = []
        for (xc, yc, thc, _) in candidates:
            xs = np.arange(xc - search_xy_m, xc + search_xy_m + 1e-12, step_xy)
            ys = np.arange(yc - search_xy_m, yc + search_xy_m + 1e-12, step_xy)
            ths = np.arange(thc - search_th_rad, thc + search_th_rad + 1e-12, step_th)
            for x in xs:
                for y in ys:
                    for th in ths:
                        sc = score_pose(field, meta, pts_r, x, y, th, res_level)
                        new_cands.append((float(x), float(y), float(th), sc))

        # keep top-K
        new_cands.sort(key=lambda t: t[3], reverse=True)
        candidates = new_cands[:top_k]

        # tighten search window for next (finer) level
        search_xy_m *= 0.5
        search_th_rad *= 0.5

    best = max(candidates, key=lambda t: t[3])
    return best  # x,y,theta,score


def main() -> None:
    np.random.seed(7)
    occ, meta = make_synthetic_map(res=0.05, size_m=10.0)
    field = likelihood_field_from_occ(occ, sigma_m=0.10, res=meta["resolution"])
    field_pyr = build_pyramid(field, levels=3)

    # True pose and an initial guess
    T_true = se2_from_xytheta(2.4, 3.2, math.radians(18.0))
    pts_r = simulate_scan_endpoints(occ, meta, T_true, max_range=6.0, noise_std=0.01)

    # Odometry-like initial guess (biased)
    x0, y0, th0 = 2.55, 3.05, math.radians(10.0)

    x, y, th, sc = multires_correlative_match(
        field_pyr, meta, pts_r, x0, y0, th0,
        search_xy_m=0.6, search_th_rad=math.radians(12),
        step_xy_m=0.05, step_th_rad=math.radians(1.0),
        top_k=60
    )

    print("True pose:     x=%.3f y=%.3f th=%.2f deg" % (T_true[0,2], T_true[1,2], math.degrees(math.atan2(T_true[1,0], T_true[0,0]))))
    print("Initial guess: x=%.3f y=%.3f th=%.2f deg" % (x0, y0, math.degrees(th0)))
    print("Matched pose:  x=%.3f y=%.3f th=%.2f deg  score=%.2f" % (x, y, math.degrees(th), sc))


if __name__ == "__main__":
    main()
