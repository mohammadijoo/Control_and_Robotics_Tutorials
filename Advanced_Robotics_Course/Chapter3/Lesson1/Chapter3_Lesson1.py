import numpy as np

def success_probability(num_samples, num_trials=1000, dim=4, goal_radius=0.1):
    """
    Approximate P(success) that at least one of num_samples
    falls inside a d-dimensional ball of radius goal_radius
    centered at 0.5 (approx. 'goal tube' in C-space).
    """
    center = 0.5 * np.ones(dim)
    successes = 0

    for _ in range(num_trials):
        q = np.random.rand(num_samples, dim)  # uniform in [0,1]^dim
        # Euclidean distance to center
        dist = np.linalg.norm(q - center, axis=1)
        if np.any(dist <= goal_radius):
            successes += 1

    return successes / num_trials

for n in [10, 50, 100, 200, 500, 1000]:
    print(f"n = {n}, approx P(success) = {success_probability(n):.3f}")
      
