import numpy as np

def random_unit_vectors(num_dirs: int, dim: int = 3) -> np.ndarray:
    """Sample unit vectors uniformly by normalizing Gaussian samples."""
    v = np.random.normal(size=(num_dirs, dim))
    v /= np.linalg.norm(v, axis=1, keepdims=True) + 1e-12
    return v  # shape (num_dirs, dim)

def ferrari_canny_approx(W: np.ndarray, num_dirs: int = 64) -> float:
    """
    Approximate Ferrari-Canny quality for planar wrenches.
    W: shape (3, m) with each column a wrench vector.
    """
    dirs = random_unit_vectors(num_dirs, dim=3)  # candidate disturbance directions
    q_val = np.inf
    for d in dirs:
        supports = d @ W  # inner products with all wrenches
        q_dir = supports.max()
        if q_dir < q_val:
            q_val = q_dir
    return float(q_val)

def robust_quality_mc(W0: np.ndarray,
                      sigma_w: float = 0.05,
                      num_samples: int = 200,
                      delta: float = 0.1):
    """
    Monte Carlo robust grasp quality.
    Returns (expected_quality, delta_quantile_quality).
    """
    qualities = []
    for _ in range(num_samples):
        noise = sigma_w * np.random.normal(size=W0.shape)
        W = W0 + noise
        q = ferrari_canny_approx(W)
        qualities.append(q)
    qualities = np.array(qualities)
    q_exp = float(qualities.mean())
    q_delta = float(np.quantile(qualities, delta))  # empirical delta-quantile
    return q_exp, q_delta

if __name__ == "__main__":
    np.random.seed(0)

    # Example nominal wrenches (3 x 4) for a simple planar 4-contact grasp
    W0 = np.array([
        [ 1.0, -1.0,  0.8, -0.8],
        [ 0.8,  0.8, -0.8, -0.8],
        [ 0.2, -0.2,  0.3, -0.3]
    ])

    q_exp, q_delta = robust_quality_mc(W0, sigma_w=0.05, num_samples=500, delta=0.1)
    print("Expected quality:", q_exp)
    print("0.1-quantile robust quality:", q_delta)
      
