import numpy as np

# State indices: 0=straight, 1=slightly entangled, 2=heavily entangled
S = 3
A = 2  # 0=pull, 1=shake
O = 2  # 0="good tension", 1="bad tension"

gamma = 0.95

# Transition model T[s, a, s']
T = np.zeros((S, A, S))
# Pull tends to improve state but risks damage from heavily entangled
T[0, 0] = [0.9, 0.1, 0.0]
T[1, 0] = [0.6, 0.3, 0.1]
T[2, 0] = [0.1, 0.4, 0.5]

# Shake tends to randomize towards slightly entangled
T[0, 1] = [0.7, 0.3, 0.0]
T[1, 1] = [0.2, 0.6, 0.2]
T[2, 1] = [0.0, 0.7, 0.3]

# Observation model Z[a, s', o]
Z = np.zeros((A, S, O))
# "Good tension" more likely when state is straight or slightly entangled
Z[:, 0, :] = [[0.9, 0.1],  # after pull
              [0.8, 0.2]]  # after shake
Z[:, 1, :] = [[0.7, 0.3],
              [0.6, 0.4]]
Z[:, 2, :] = [[0.3, 0.7],
              [0.4, 0.6]]

# Reward R[s, a]: prefer straight cable, penalize heaviness
R = np.array([
    [+1.0, +0.5],   # straight
    [-0.2, -0.1],   # slightly entangled
    [-1.0, -0.5],   # heavily entangled
])

def belief_predict(b, a):
    """One-step prediction b'(s') = sum_s T[s,a,s'] b(s)."""
    return T[:, a, :].T.dot(b)

def belief_update(b, a, o):
    """Bayes update using observation o after action a."""
    bp = belief_predict(b, a)  # predictive belief
    # likelihood p(o | s', a)
    lik = Z[a, :, o]
    num = lik * bp
    if np.sum(num) < 1e-12:
        # Numerical safeguard
        return np.ones_like(b) / len(b)
    return num / np.sum(num)

def value_iteration_mdp(max_iter=1000, tol=1e-6):
    """Solve fully observable MDP for QMDP."""
    V = np.zeros(S)
    for it in range(max_iter):
        Q = R + gamma * np.einsum("sasp->sa", T * V[np.newaxis, np.newaxis, :])
        V_new = np.max(Q, axis=1)
        if np.max(np.abs(V_new - V)) < tol:
            break
        V = V_new
    Q = R + gamma * np.einsum("sasp->sa", T * V[np.newaxis, np.newaxis, :])
    return V, Q

V_mdp, Q_mdp = value_iteration_mdp()

def qmdp_action(b):
    """QMDP action: maximize expected Q(s,a) under belief b."""
    # expected Q for each action
    Qa = b.dot(Q_mdp)  # shape (A,)
    return int(np.argmax(Qa))

# Example usage
b = np.array([1/3, 1/3, 1/3], dtype=float)  # initial belief
obs_seq = [0, 1, 0]  # hypothetical tension measurements

for t, o in enumerate(obs_seq):
    a = qmdp_action(b)
    print(f"t={t}, belief={b}, action={a}")
    b = belief_update(b, a, o)

print("Final belief:", b)
      
