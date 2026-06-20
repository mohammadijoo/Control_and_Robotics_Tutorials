import numpy as np

# Discretized state space for 1D "pose"
n_theta = 360
theta_grid = np.linspace(-np.pi, np.pi, n_theta, endpoint=False)

def softplus(x):
    return np.log1p(np.exp(x))

def logistic(u):
    return 1.0 / (1.0 + np.exp(-u))

def measurement_likelihood(z, theta, a, kappa=4.0):
    """
    Binary measurement model p(z | theta, a).
    z in {0,1}, theta and a are scalars.
    """
    p1 = logistic(kappa * np.cos(theta - a))
    return p1 if z == 1 else (1.0 - p1)

def normalize(b):
    s = np.sum(b)
    if s == 0.0:
        return np.ones_like(b) / float(b.size)
    return b / s

def entropy(b):
    # Natural-log entropy; ignore zero entries
    mask = b > 0.0
    return -np.sum(b[mask] * np.log(b[mask]))

def expected_entropy_after_action(b, a, kappa=4.0):
    """
    Compute E_z[ H(b') ] under action a (myopic).
    z is binary {0,1}.
    """
    # Predictive likelihood of z
    p_z = []
    posteriors = []
    for z in [0, 1]:
        # Unnormalized posterior
        like = np.array([
            measurement_likelihood(z, th, a, kappa) for th in theta_grid
        ])
        b_unnorm = like * b
        pz = np.sum(b_unnorm)
        p_z.append(pz)
        posteriors.append(normalize(b_unnorm) if pz > 0 else b.copy())
    p_z = np.array(p_z)
    # Expected posterior entropy
    H = 0.0
    for z in [0, 1]:
        if p_z[z] > 0.0:
            H += p_z[z] * entropy(posteriors[z])
    return H

def choose_action(b, candidate_views, kappa=4.0):
    H_prior = entropy(b)
    best_a = None
    best_ig = -np.inf
    for a in candidate_views:
        H_post = expected_entropy_after_action(b, a, kappa)
        ig = H_prior - H_post
        if ig > best_ig:
            best_ig = ig
            best_a = a
    return best_a, best_ig

def update_belief(b, a, z, kappa=4.0):
    like = np.array([
        measurement_likelihood(z, th, a, kappa) for th in theta_grid
    ])
    return normalize(like * b)

# Example loop: start uniform, run a few active sensing steps
b = np.ones(n_theta) / float(n_theta)
candidate_views = np.linspace(-np.pi, np.pi, 12, endpoint=False)

# Ground-truth theta for simulation
theta_true = 0.7

rng = np.random.default_rng(0)

for t in range(6):
    a_star, ig = choose_action(b, candidate_views)
    # Simulate measurement from true state
    p1 = measurement_likelihood(1, theta_true, a_star)
    z = 1 if rng.random() < p1 else 0
    b = update_belief(b, a_star, z)
    print(f"Step {t}: a = {a_star:.2f}, z = {z}, IG = {ig:.3f}, H = {entropy(b):.3f}")
      
