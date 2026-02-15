import numpy as np

# Discrete states: 0 = near door, 1 = far from door
S = [0, 1]

# Actions and observations
A = ["stay", "step"]
O = ["door", "no-door"]

# Transition matrices T[a][s, s_prime]
T = {
    "stay": np.array([
        [0.9, 0.0],
        [0.1, 1.0]
    ]),
    "step": np.array([
        [0.1, 0.0],
        [0.9, 1.0]
    ])
}

# Observation model as vectors Z[o][s_prime]
Z = {
    "door": np.array([0.2, 0.9]),
    "no-door": np.array([0.8, 0.1])
}

# Reward R[s, a]
R = {
    "stay": np.array([-0.05, -0.05]),   # small time penalty
    "step": np.array([-0.5,  +1.0])     # costly motion, reward for being at door
}

gamma = 0.95

def normalize(p):
    total = float(np.sum(p))
    if total == 0.0:
        return np.ones_like(p) / float(len(p))
    return p / total

def predict_belief(b, a):
    # b is a length-2 numpy array
    # T[a] has shape (2, 2) with rows indexed by s and columns by s_prime
    # We want p(s_prime) = sum_s T[s, s_prime] * b[s]
    return T[a].T @ b

def belief_update(b, a, o):
    b_pred = predict_belief(b, a)
    b_post = Z[o] * b_pred
    return normalize(b_post)

def expected_reward(b, a):
    return float(np.dot(b, R[a]))

def observation_prob(b, a, o):
    b_pred = predict_belief(b, a)
    return float(np.dot(b_pred, Z[o]))

def value(b, depth):
    """
    Simple finite-horizon value function via recursion.
    depth = remaining planning steps.
    """
    if depth == 0:
        return 0.0
    q_values = []
    for a in A:
        r = expected_reward(b, a)
        future = 0.0
        b_pred = predict_belief(b, a)
        for o in O:
            # probability of observation o
            p_o = float(np.dot(b_pred, Z[o]))
            if p_o == 0.0:
                continue
            # posterior given o
            b_post = normalize(Z[o] * b_pred)
            future += p_o * value(b_post, depth - 1)
        q_values.append(r + gamma * future)
    return float(np.max(q_values))

def greedy_action(b, depth):
    """
    One-step lookahead using value() for depth - 1 steps in the future.
    """
    best_a = None
    best_q = None
    for a in A:
        r = expected_reward(b, a)
        future = 0.0
        b_pred = predict_belief(b, a)
        for o in O:
            p_o = float(np.dot(b_pred, Z[o]))
            if p_o == 0.0:
                continue
            b_post = normalize(Z[o] * b_pred)
            future += p_o * value(b_post, depth - 1)
        q = r + gamma * future
        if best_q is None:
            best_q = q
            best_a = a
        else:
            best_q = max(best_q, q)
            if best_q == q:
                best_a = a
    return best_a

if __name__ == "__main__":
    # Start from uniform belief
    b = np.array([0.5, 0.5])
    horizon = 3
    a_star = greedy_action(b, horizon)
    print("Greedy action at initial belief:", a_star)
      
