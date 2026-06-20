import numpy as np

def build_K(num_points):
    """
    Build 1D smoothness matrix K for internal points using
    second-order finite differences. For 2D we will apply it per dimension.
    """
    K = np.zeros((num_points, num_points))
    for k in range(num_points):
        if k > 0:
            K[k, k-1] -= 1.0
            K[k-1, k] -= 1.0
        K[k, k] += 2.0
    # This approximates integral of squared second derivative up to scaling.
    return K

def obstacle_cost(x):
    """Simple radial obstacle around origin."""
    d = np.linalg.norm(x)
    d0 = 0.5  # obstacle radius
    if d < d0:
        return 0.5 * (d0 - d)**2
    return 0.0

def obstacle_grad(x):
    """Gradient of obstacle_cost."""
    d = np.linalg.norm(x)
    d0 = 0.5
    if d < 1e-8:
        return np.zeros_like(x)
    if d < d0:
        return - (d0 - d) * (x / d)
    return np.zeros_like(x)

def chomp_step(q, K, alpha=0.1, lam=1.0):
    """
    q: array of shape (N, 2) internal waypoints (2D point robot).
    K: N x N smoothness matrix.
    """
    N = q.shape[0]

    # Smoothness gradient: flatten and apply K kron I_2
    q_flat = q.reshape(N * 2)
    K_big = np.kron(K, np.eye(2))
    grad_smooth = K_big @ q_flat

    # Obstacle gradient: per waypoint
    grad_obs = np.zeros_like(q)
    for k in range(N):
        grad_obs[k, :] = obstacle_grad(q[k, :])
    grad_obs_flat = grad_obs.reshape(N * 2)

    grad_total = grad_smooth + lam * grad_obs_flat

    # Covariant update: solve K_big * delta = grad_total
    delta = np.linalg.solve(K_big + 1e-6 * np.eye(N * 2), grad_total)
    q_new_flat = q_flat - alpha * delta
    return q_new_flat.reshape(N, 2)

def stomp_step(q, K, num_rollouts=20, noise_std=0.05, lam=1.0, eta=0.1):
    """
    STOMP step around current trajectory q (N x 2).
    """
    N = q.shape[0]
    q_flat = q.reshape(N * 2)

    # Shape noise using smoothness inverse K^{-1}
    K_big = np.kron(K, np.eye(2))
    K_inv = np.linalg.inv(K_big + 1e-6 * np.eye(N * 2))

    epsilons = []
    costs = []
    for m in range(num_rollouts):
        # Sample white noise then smooth it
        e_raw = np.random.randn(N * 2) * noise_std
        e_smooth = K_inv @ e_raw
        q_pert = (q_flat + e_smooth).reshape(N, 2)

        # Compute cost (smoothness + obstacle)
        smooth = 0.5 * float(q_pert.reshape(N * 2).T @ (K_big @ q_pert.reshape(N * 2)))
        obs = 0.0
        for k in range(N):
            obs += obstacle_cost(q_pert[k, :])
        cost = smooth + lam * obs

        epsilons.append(e_smooth)
        costs.append(cost)

    costs = np.array(costs)
    # Soft-min weights
    c_min = np.min(costs)
    weights = np.exp(-(costs - c_min) / eta)
    weights /= np.sum(weights)

    delta = np.zeros_like(q_flat)
    for w, e in zip(weights, epsilons):
        delta += w * e

    q_new_flat = q_flat + delta
    return q_new_flat.reshape(N, 2)

# Example usage:
if __name__ == "__main__":
    np.random.seed(0)
    N = 30  # internal waypoints
    K = build_K(N)

    start = np.array([-1.0, 0.0])
    goal = np.array([1.0, 0.0])
    # Initialize straight line
    alphas = np.linspace(0.0, 1.0, N + 2)[1:-1]
    q = np.outer(1 - alphas, start) + np.outer(alphas, goal)

    for it in range(50):
        q = chomp_step(q, K, alpha=0.05, lam=5.0)
    # Alternatively:
    # for it in range(50):
    #     q = stomp_step(q, K)
      
