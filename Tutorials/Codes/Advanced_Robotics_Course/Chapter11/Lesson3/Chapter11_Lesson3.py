import numpy as np

# Small gridworld as proxy for a low-res C-space
side = 3
n_states = side * side
n_actions = 4  # 0: up, 1: right, 2: down, 3: left
gamma = 0.9

def state_to_rc(s):
    return divmod(s, side)

def rc_to_state(r, c):
    return r * side + c

def make_transition():
    P = np.zeros((n_states, n_actions, n_states))
    for s in range(n_states):
        r, c = state_to_rc(s)
        for a in range(n_actions):
            nr, nc = r, c
            if a == 0:
                nr = max(r - 1, 0)
            elif a == 1:
                nc = min(c + 1, side - 1)
            elif a == 2:
                nr = min(r + 1, side - 1)
            else:
                nc = max(c - 1, 0)
            s_next = rc_to_state(nr, nc)
            P[s, a, s_next] = 1.0
    return P

P = make_transition()

# Feature map: one-hot over states (k = n_states)
k = n_states
phi = np.eye(n_states)  # phi[s, :] is feature vector for state s

def empirical_feature_counts(demos):
    """demos: list of trajectories, each as list of states"""
    mu_E = np.zeros(k)
    for traj in demos:
        for t, s in enumerate(traj):
            mu_E += (gamma ** t) * phi[s]
    mu_E /= len(demos)
    return mu_E

def soft_value_iteration(theta, n_iters=50):
    # reward only depends on state here: R(s) = theta^T phi(s) = theta[s]
    R = theta
    V = np.zeros(n_states)
    for _ in range(n_iters):
        Q = np.zeros((n_states, n_actions))
        for s in range(n_states):
            for a in range(n_actions):
                Q[s, a] = R[s] + gamma * np.dot(P[s, a], V)
        # soft value function
        V = np.log(np.exp(Q).sum(axis=1) + 1e-8)
    # derive policy
    Q = np.zeros((n_states, n_actions))
    for s in range(n_states):
        for a in range(n_actions):
            Q[s, a] = R[s] + gamma * np.dot(P[s, a], V)
    pi = np.exp(Q - V[:, None])
    pi /= pi.sum(axis=1, keepdims=True)
    return V, pi

def state_visitation(pi, start_dist, horizon=20):
    d_s = np.zeros(n_states)
    d = start_dist.copy()
    for t in range(horizon):
        d_s += (gamma ** t) * d
        d_next = np.zeros(n_states)
        for s in range(n_states):
            for a in range(n_actions):
                for s2 in range(n_states):
                    d_next[s2] += d[s] * pi[s, a] * P[s, a, s2]
        d = d_next
    d_s *= (1.0 - gamma)
    return d_s

def expected_features(theta, start_dist):
    _, pi = soft_value_iteration(theta)
    d_s = state_visitation(pi, start_dist)
    return d_s @ phi  # (n_states,) @ (n_states, k) = (k,)

# Example expert demonstrations (hand-designed "right then down" strategy)
demos = [
    [0, 1, 2, 5, 8],   # trajectory 1
    [1, 2, 5, 8],      # trajectory 2
]

mu_E = empirical_feature_counts(demos)

theta = np.zeros(k)
start_dist = np.zeros(n_states)
start_dist[0] = 1.0  # fixed start state

alpha = 0.5
for itr in range(50):
    mu_theta = expected_features(theta, start_dist)
    grad = mu_E - mu_theta
    theta += alpha * grad  # gradient ascent on MaxEnt IRL objective

print("Learned reward (per state):")
print(theta.reshape((side, side)))
      
