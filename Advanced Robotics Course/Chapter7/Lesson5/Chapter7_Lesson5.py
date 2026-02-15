import numpy as np

def discretize_friction_cone(normal, mu, k=8):
    """
    Approximate a 3D Coulomb friction cone by k unit force directions.
    normal : (3,) array (unit vector)
    mu     : scalar friction coefficient
    """
    n = normal / np.linalg.norm(normal)
    # Build orthonormal basis {t1, t2, n}
    # Pick arbitrary vector not parallel to n
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(tmp, n)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    t1 = np.cross(n, tmp)
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)

    dirs = []
    angles = np.linspace(0.0, 2.0 * np.pi, k, endpoint=False)
    for theta in angles:
        # Tangential direction in the friction circle
        t = np.cos(theta) * t1 + np.sin(theta) * t2
        # Construct a direction on the cone boundary
        # f = n + mu * t, then normalize
        f = n + mu * t
        f /= np.linalg.norm(f)
        dirs.append(f)
    return np.stack(dirs, axis=0)  # shape (k, 3)


def contact_wrenches(p, n, mu, k=8, wrench_scale=1.0):
    """
    Generate 6D wrenches for a single contact.
    p : (3,) contact position in object frame
    n : (3,) outward normal (unit vector)
    mu: friction coefficient
    Returns: array (k, 6)
    """
    ray_dirs = discretize_friction_cone(n, mu, k)
    wrenches = []
    for f_dir in ray_dirs:
        # Scale to max normal force = wrench_scale
        # Here we just use wrench_scale as a normalization constant.
        f = wrench_scale * f_dir
        m = np.cross(p, f)  # moment at origin
        w = np.concatenate([f, m])
        wrenches.append(w)
    return np.stack(wrenches, axis=0)


def build_wrench_set(contacts, k=8, mu_default=0.8):
    """
    contacts: list of dicts with keys:
      'p': np.array(3,), 'n': np.array(3,), optional 'mu'
    Returns: array (M, 6) of all contact wrenches.
    """
    all_w = []
    for c in contacts:
        p = c["p"]
        n = c["n"]
        mu = c.get("mu", mu_default)
        Wc = contact_wrenches(p, n, mu, k=k)
        all_w.append(Wc)
    return np.vstack(all_w)


def sample_unit_directions(dim, num_samples):
    """
    Sample random unit vectors in R^dim using Gaussian normalization.
    """
    X = np.random.normal(size=(num_samples, dim))
    X /= np.linalg.norm(X, axis=1, keepdims=True)
    return X  # shape (num_samples, dim)


def epsilon_quality(wrenches, num_directions=256):
    """
    Approximate Ferrari-Canny epsilon quality.
    wrenches: array (M, 6)
    """
    if wrenches.shape[0] == 0:
        return 0.0
    dirs = sample_unit_directions(6, num_directions)  # u_j
    # Compute support values: for each direction, max(u_j^T w)
    # wrenches.T has shape (6, M)
    proj = dirs.dot(wrenches.T)  # shape (num_directions, M)
    support_vals = np.max(proj, axis=1)  # max over w
    # Epsilon approx is min over directions
    return float(np.min(support_vals))


def score_grasp(contacts, k=8, mu_default=0.8, num_directions=256):
    """
    High-level: contacts -> epsilon grasp score.
    """
    W = build_wrench_set(contacts, k=k, mu_default=mu_default)
    return epsilon_quality(W, num_directions=num_directions)


if __name__ == "__main__":
    # Example: simple planar-ish three-finger grasp around the origin
    contacts = [
        {"p": np.array([0.05, 0.0, 0.0]), "n": np.array([-1.0, 0.0, 0.0]), "mu": 0.7},
        {"p": np.array([-0.05, 0.04, 0.0]), "n": np.array([1.0, -1.0, 0.0]), "mu": 0.7},
        {"p": np.array([-0.05, -0.04, 0.0]), "n": np.array([1.0, 1.0, 0.0]), "mu": 0.7},
    ]
    # Normalize normals
    for c in contacts:
        c["n"] = c["n"] / np.linalg.norm(c["n"])

    q = score_grasp(contacts, k=8, mu_default=0.7, num_directions=512)
    print("Approximate epsilon quality:", q)
      
