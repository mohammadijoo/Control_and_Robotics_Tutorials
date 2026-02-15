import numpy as np

def ring_neighbors(N):
    """Return neighbor sets for a ring topology."""
    neighbors = []
    for i in range(N):
        left = (i - 1) % N
        right = (i + 1) % N
        neighbors.append([left, right])
    return neighbors

def centralized_step(p, k_gain, dt):
    """One step of centralized rendezvous: move toward initial average."""
    N = len(p)
    p_avg0 = np.mean(p0)  # global average at time 0 (constant)
    u = -k_gain * (p - p_avg0)
    return p + dt * u

def decentralized_step(p, neighbors, k_gain, dt):
    """One step of decentralized neighbor-averaging."""
    N = len(p)
    u = np.zeros_like(p)
    for i in range(N):
        s = 0.0
        for j in neighbors[i]:
            s += (p[i] - p[j])
        u[i] = -k_gain * s
    return p + dt * u

# parameters
N = 6
dt = 0.05
k_gain = 1.0
T = 5.0
steps = int(T / dt)

# random initial positions
rng = np.random.default_rng(1)
p0 = rng.uniform(-5.0, 5.0, size=N)

# neighbor structure (ring)
nbrs = ring_neighbors(N)

# simulate both controllers
p_central = p0.copy()
p_decent  = p0.copy()

hist_central = [p_central.copy()]
hist_decent  = [p_decent.copy()]

for k in range(steps):
    p_central = centralized_step(p_central, k_gain, dt)
    p_decent  = decentralized_step(p_decent, nbrs, k_gain, dt)
    hist_central.append(p_central.copy())
    hist_decent.append(p_decent.copy())

hist_central = np.array(hist_central)
hist_decent  = np.array(hist_decent)

# Example: print final positions
print("Initial positions:", p0)
print("Centralized final:", hist_central[-1])
print("Decentralized final:", hist_decent[-1])
print("Initial average:", np.mean(p0))
      
