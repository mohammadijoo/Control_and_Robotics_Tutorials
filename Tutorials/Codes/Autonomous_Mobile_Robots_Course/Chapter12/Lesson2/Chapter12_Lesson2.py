# Chapter12_Lesson2.py
# Loop Closure Detection Concepts — minimal research-grade demo (descriptor retrieval + geometric gating)
# Author: Course scaffold generator

import numpy as np

try:
    from sklearn.neighbors import NearestNeighbors
except Exception as e:
    raise ImportError("This demo needs scikit-learn (pip install scikit-learn).") from e

try:
    from scipy.stats import chi2
except Exception as e:
    raise ImportError("This demo needs SciPy (pip install scipy).") from e


def make_square_loop(N: int = 400, side: float = 20.0):
    """
    Ground-truth 2D loop: a square (0,0)->(side,0)->(side,side)->(0,side)->(0,0)
    Returns:
        gt: (N,3) pose array [x,y,theta]
    """
    # Parameterize by arclength
    perim = 4.0 * side
    s = np.linspace(0.0, perim, N, endpoint=False)
    gt = np.zeros((N, 3), dtype=float)

    for i, si in enumerate(s):
        seg = int(si // side)  # 0..3
        u = (si % side)
        if seg == 0:
            x, y, th = u, 0.0, 0.0
        elif seg == 1:
            x, y, th = side, u, np.pi / 2
        elif seg == 2:
            x, y, th = side - u, side, np.pi
        else:
            x, y, th = 0.0, side - u, -np.pi / 2
        gt[i] = [x, y, th]
    return gt


def add_odometry_drift(gt, sigma_step_xy=0.03, sigma_step_th=0.003, drift_xy=(0.002, -0.001), drift_th=0.0002):
    """
    Build a drifting odometry trajectory by integrating noisy increments.
    """
    N = gt.shape[0]
    odo = np.zeros_like(gt)
    odo[0] = gt[0].copy()
    rng = np.random.default_rng(0)

    def wrap(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    for k in range(1, N):
        d = gt[k] - gt[k - 1]
        # keep theta increment wrapped
        d[2] = wrap(d[2])

        noise = np.array([
            rng.normal(0.0, sigma_step_xy),
            rng.normal(0.0, sigma_step_xy),
            rng.normal(0.0, sigma_step_th),
        ])
        drift = np.array([drift_xy[0], drift_xy[1], drift_th])
        inc = d + noise + drift
        odo[k] = odo[k - 1] + inc
        odo[k, 2] = wrap(odo[k, 2])
    return odo


def build_prototypes(V=300, n_places=80, words_per_place=18, seed=1):
    """
    Each 'place' has a sparse word-prototype over a vocabulary of size V.
    """
    rng = np.random.default_rng(seed)
    prot = np.zeros((n_places, V), dtype=float)
    for p in range(n_places):
        idx = rng.choice(V, size=words_per_place, replace=False)
        w = rng.uniform(0.5, 2.0, size=words_per_place)
        prot[p, idx] = w
    # normalize prototypes
    prot = prot / (np.linalg.norm(prot, axis=1, keepdims=True) + 1e-12)
    return prot


def assign_place_id(gt_xy, cell=3.0, n_places=80):
    """
    Quantize position to a place id (toy perceptual aliasing included via mod).
    """
    q = np.floor(gt_xy / cell).astype(int)
    # simple hash -> id in [0, n_places)
    pid = (q[:, 0] * 73856093 + q[:, 1] * 19349663) % n_places
    return pid.astype(int)


def tfidf_from_counts(C):
    """
    C: (N,V) count matrix
    returns normalized TF-IDF matrix (N,V)
    """
    N, V = C.shape
    df = (C > 0).sum(axis=0)  # document frequency
    idf = np.log((N + 1.0) / (df + 1.0)) + 1.0
    tf = C / (C.sum(axis=1, keepdims=True) + 1e-12)
    X = tf * idf
    X = X / (np.linalg.norm(X, axis=1, keepdims=True) + 1e-12)
    return X


def cosine_retrieval(X, min_sep=30, k=5):
    """
    For each i, retrieve up to k candidates among frames [0, i-min_sep).
    Returns list of (i,j,sim) candidates.
    """
    N = X.shape[0]
    candidates = []
    for i in range(N):
        jmax = i - min_sep
        if jmax <= 0:
            continue
        nbrs = NearestNeighbors(n_neighbors=min(k, jmax), metric="cosine")
        nbrs.fit(X[:jmax])
        dist, idx = nbrs.kneighbors(X[i:i+1], return_distance=True)
        for d, j in zip(dist.ravel(), idx.ravel()):
            sim = 1.0 - float(d)
            candidates.append((i, int(j), sim))
    return candidates


def gaussian_llr(sim, mu1=0.82, s1=0.06, mu0=0.35, s0=0.10, eps=1e-12):
    """
    Log-likelihood ratio log p(sim|H1)/p(sim|H0) for two 1D Gaussians.
    """
    # log N(x;mu,s^2) = -0.5*log(2pi s^2) - 0.5*(x-mu)^2/s^2
    x = sim
    ll1 = -0.5*np.log(2*np.pi*(s1**2)+eps) - 0.5*((x-mu1)**2)/(s1**2+eps)
    ll0 = -0.5*np.log(2*np.pi*(s0**2)+eps) - 0.5*((x-mu0)**2)/(s0**2+eps)
    return float(ll1 - ll0)


def geometric_gate(odo, gt, i, j, sigma_xy=0.20, sigma_th=0.07, alpha=0.995):
    """
    Toy geometric verification:
      - simulate a loop closure measurement z_ij from gt
      - compare against predicted relative pose from odometry
      - chi-square gate on residual
    """
    def wrap(a):
        return (a + np.pi) % (2*np.pi) - np.pi

    # predicted from odometry (what the estimator believes before loop closure)
    pred = odo[i] - odo[j]
    pred[2] = wrap(pred[2])

    # 'scan matching' measurement around ground-truth relative pose
    rng = np.random.default_rng(i * 1000 + j)
    z = (gt[i] - gt[j]) + np.array([rng.normal(0, sigma_xy), rng.normal(0, sigma_xy), rng.normal(0, sigma_th)])
    z[2] = wrap(z[2])

    r = z - pred
    r[2] = wrap(r[2])
    S = np.diag([sigma_xy**2, sigma_xy**2, sigma_th**2])
    d2 = float(r.T @ np.linalg.inv(S) @ r)
    thr = float(chi2.ppf(alpha, df=3))
    return (d2 < thr), d2, thr


def evaluate():
    np.random.seed(0)
    N = 420
    gt = make_square_loop(N=N, side=25.0)
    odo = add_odometry_drift(gt)

    # build descriptors (BoW counts from place prototypes)
    V = 300
    prot = build_prototypes(V=V, n_places=90, words_per_place=20)
    pid = assign_place_id(gt[:, :2], cell=3.2, n_places=prot.shape[0])

    rng = np.random.default_rng(2)
    C = np.zeros((N, V), dtype=float)
    for i in range(N):
        base = prot[pid[i]]
        # sample counts: higher on prototype words, plus clutter (perceptual aliasing)
        lam = 8.0 * base + 0.15
        C[i] = rng.poisson(lam)
        # random dropout to imitate viewpoint change
        drop = rng.uniform(0, 1, size=V) < 0.02
        C[i, drop] = 0.0

    X = tfidf_from_counts(C)

    # ground-truth loop closures: far in time, close in space
    min_sep = 40
    radius = 1.4
    gt_loops = set()
    for i in range(N):
        for j in range(0, i - min_sep):
            if np.linalg.norm(gt[i, :2] - gt[j, :2]) < radius:
                gt_loops.add((i, j))

    # candidate retrieval + LLR test + geometric gate
    cands = cosine_retrieval(X, min_sep=min_sep, k=6)

    detections = set()
    scored = []
    for (i, j, sim) in cands:
        llr = gaussian_llr(sim)
        # Bayesian decision threshold (log eta); tune for low false positives
        log_eta = np.log(50.0)  # favor H0 unless strong evidence
        if llr > log_eta:
            ok, d2, thr = geometric_gate(odo, gt, i, j)
            if ok:
                detections.add((i, j))
                scored.append((i, j, sim, llr, d2, thr))

    tp = len(detections.intersection(gt_loops))
    fp = len(detections - gt_loops)
    fn = len(gt_loops - detections)
    prec = tp / (tp + fp + 1e-12)
    rec = tp / (tp + fn + 1e-12)

    print("N =", N)
    print("GT loop edges:", len(gt_loops))
    print("Detected loop edges:", len(detections))
    print(f"Precision: {prec:.3f}  Recall: {rec:.3f}")
    if scored:
        scored = sorted(scored, key=lambda t: -t[2])
        print("\nTop detections (i,j,sim,llr,d2,thr):")
        for row in scored[:10]:
            print(row)

    return prec, rec


if __name__ == "__main__":
    evaluate()
