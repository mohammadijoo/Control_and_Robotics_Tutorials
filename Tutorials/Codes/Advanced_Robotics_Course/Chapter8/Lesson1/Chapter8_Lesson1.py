import numpy as np
from numpy.linalg import norm

# ----- Part 1: toy grasp map G (6 x p) -----
def random_wrench():
    # Random contact wrench with limited magnitude
    w = np.random.randn(6)
    return w / norm(w)

def build_grasp_map(num_contacts=4, rays_per_contact=4, seed=0):
    rng = np.random.RandomState(seed)
    W = []
    for i in range(num_contacts):
        for r in range(rays_per_contact):
            w = rng.randn(6)
            w = w / np.linalg.norm(w)
            W.append(w)
    G = np.stack(W, axis=1)  # 6 x p
    return G

# ----- Part 2: approximate epsilon-quality by sampling directions -----
def epsilon_quality(G, num_dirs=200):
    # G: 6 x p matrix of primitive wrenches
    p = G.shape[1]
    eps = np.inf
    for _ in range(num_dirs):
        v = np.random.randn(6)
        v = v / norm(v)
        # support function h(v) = max_j v^T w_j
        support = np.max(v.dot(G))
        eps = min(eps, support)
    return float(max(eps, 0.0))

# ----- Part 3: build synthetic dataset for calibration -----
def synthetic_dataset(num_samples=200):
    X_feat = []
    y = []
    for i in range(num_samples):
        G = build_grasp_map(seed=i)
        q = epsilon_quality(G)
        # "Ground-truth" success probability is a saturating function of q
        p_success = 1.0 / (1.0 + np.exp(-5.0 * (q - 0.2)))
        label = np.random.rand() < p_success
        X_feat.append([q])
        y.append(1 if label else 0)
    return np.array(X_feat), np.array(y)

# ----- Part 4: logistic regression training (1D feature) -----
def train_logistic_regression(X, y, lr=0.5, epochs=200):
    # X: (N,1), y: (N,)
    w = 0.0
    b = 0.0
    for _ in range(epochs):
        z = w * X[:, 0] + b
        p = 1.0 / (1.0 + np.exp(-z))
        grad_w = -np.mean((y - p) * X[:, 0])
        grad_b = -np.mean(y - p)
        w -= lr * grad_w
        b -= lr * grad_b
    return w, b

def predict_success_prob(q, w, b):
    z = w * q + b
    return 1.0 / (1.0 + np.exp(-z))

if __name__ == "__main__":
    X, y = synthetic_dataset(num_samples=300)
    w, b = train_logistic_regression(X, y)
    test_G = build_grasp_map(seed=999)
    q_test = epsilon_quality(test_G)
    p_hat = predict_success_prob(q_test, w, b)
    print("Analytical quality q =", q_test)
    print("Predicted success probability =", p_hat)
      
