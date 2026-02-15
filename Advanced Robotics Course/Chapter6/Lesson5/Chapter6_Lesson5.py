import numpy as np

# Small grid world: states 0,...,N-1
# For robotics, think of states as coarse cells in C-space or x-y grid.
N_STATES = 5
N_ACTIONS = 2  # e.g. "left" and "right" primitives

gamma = 0.95
lambda_risk = 0.5  # entropic risk aversion

# Nominal transitions P0[s, a, s']
P0 = np.zeros((N_STATES, N_ACTIONS, N_STATES))
# Simple line chain dynamics:
for s in range(N_STATES):
    for a in range(N_ACTIONS):
        # a==0: left, a==1: right
        if a == 0:
            s_next = max(0, s - 1)
        else:
            s_next = min(N_STATES - 1, s + 1)
        P0[s, a, s_next] = 1.0

# Interval uncertainty around P0: allow some slip probability to neighbors
eps = 0.2
P_min = np.maximum(0.0, P0 - eps)
P_max = np.minimum(1.0, P0 + eps)

# Stage cost: penalize being away from goal (state N_STATES-1)
goal = N_STATES - 1
c = np.zeros((N_STATES, N_ACTIONS))
for s in range(N_STATES):
    dist = abs(goal - s)
    c[s, :] = dist  # same cost for both actions here

def robust_inner_max(V, s, a):
    """
    Compute max_p sum_s' p(s') V(s')
    subject to sum p = 1, P_min <= p <= P_max.
    Here we implement a greedy saturating algorithm since the LP is small.
    """
    v = V.copy()
    # Sort states by value descending
    order = np.argsort(-v)
    p = np.zeros_like(v)
    remaining = 1.0
    for s_prime in order:
        # allocate as much probability as possible up to P_max
        max_allow = min(P_max[s, a, s_prime], remaining)
        min_allow = P_min[s, a, s_prime]
        # ensure at least min_allow
        alloc = max(min_allow, max_allow)
        alloc = min(alloc, remaining)
        p[s_prime] = alloc
        remaining -= alloc
        if remaining <= 1e-12:
            break
    # If remaining > 0, distribute uniformly over free states within bounds
    if remaining > 1e-12:
        free = [idx for idx in range(N_STATES) if p[idx] < P_max[s, a, idx] - 1e-12]
        if free:
            extra = remaining / len(free)
            for idx in free:
                p[idx] = min(P_max[s, a, idx], p[idx] + extra)
    return float(np.dot(p, V))

def value_iteration_robust(max_iter=200, tol=1e-6):
    V = np.zeros(N_STATES)
    for it in range(max_iter):
        V_new = np.zeros_like(V)
        for s in range(N_STATES):
            q_vals = []
            for a in range(N_ACTIONS):
                worst = robust_inner_max(V, s, a)
                q_vals.append(c[s, a] + gamma * worst)
            V_new[s] = min(q_vals)
        if np.max(np.abs(V_new - V)) < tol:
            break
        V = V_new
    # greedy policy
    pi = np.zeros(N_STATES, dtype=int)
    for s in range(N_STATES):
        q_vals = []
        for a in range(N_ACTIONS):
            worst = robust_inner_max(V, s, a)
            q_vals.append(c[s, a] + gamma * worst)
        pi[s] = int(np.argmin(q_vals))
    return V, pi

def value_iteration_entropic(max_iter=200, tol=1e-6):
    V = np.zeros(N_STATES)
    for it in range(max_iter):
        V_new = np.zeros_like(V)
        for s in range(N_STATES):
            q_vals = []
            for a in range(N_ACTIONS):
                # Entropic risk-sensitive Bellman update under nominal P0
                exp_term = 0.0
                for s_prime in range(N_STATES):
                    if P0[s, a, s_prime] > 0.0:
                        exp_term += P0[s, a, s_prime] * np.exp(
                            lambda_risk * (c[s, a] + gamma * V[s_prime])
                        )
                q_vals.append((1.0 / lambda_risk) * np.log(exp_term))
            V_new[s] = min(q_vals)
        if np.max(np.abs(V_new - V)) < tol:
            break
        V = V_new
    pi = np.zeros(N_STATES, dtype=int)
    for s in range(N_STATES):
        q_vals = []
        for a in range(N_ACTIONS):
            exp_term = 0.0
            for s_prime in range(N_STATES):
                if P0[s, a, s_prime] > 0.0:
                    exp_term += P0[s, a, s_prime] * np.exp(
                        lambda_risk * (c[s, a] + gamma * V[s_prime])
                    )
            q_vals.append((1.0 / lambda_risk) * np.log(exp_term))
        pi[s] = int(np.argmin(q_vals))
    return V, pi

if __name__ == "__main__":
    V_rob, pi_rob = value_iteration_robust()
    V_risk, pi_risk = value_iteration_entropic()
    print("Robust value:", V_rob)
    print("Robust policy:", pi_rob)
    print("Risk-sensitive value:", V_risk)
    print("Risk-sensitive policy:", pi_risk)
      
