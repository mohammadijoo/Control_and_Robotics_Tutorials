import numpy as np

rng = np.random.default_rng(seed=0)

def build_ring_adjacency(N):
    A = np.zeros((N, N), dtype=float)
    for i in range(N):
        A[i, (i - 1) % N] = 1.0
        A[i, (i + 1) % N] = 1.0
    return A

def consensus_step(x, A, alive, eps=0.2, noise_std=0.0, p_drop=0.0):
    N = x.shape[0]
    # Effective adjacency with symmetric link drops
    mask = rng.random((N, N)) >= p_drop
    A_eff = A * mask * mask.T
    x_new = np.copy(x)
    for i in range(N):
        if not alive[i]:
            continue
        # Local noisy consensus update
        neighbors = np.where(A_eff[i] > 0.0)[0]
        if neighbors.size == 0:
            continue
        diff = x[neighbors] - x[i]
        x_new[i] = x[i] + eps * diff.mean() + noise_std * rng.normal()
    return x_new

def simulate_swarm(N=50, steps=200, p_fail=0.1, p_drop=0.05,
                   eps=0.2, noise_std=0.01):
    # Initial headings
    x = rng.uniform(low=-1.0, high=1.0, size=N)
    alive = np.ones(N, dtype=bool)

    A = build_ring_adjacency(N)

    # Draw failures once at the beginning
    failures = rng.random(N) < p_fail
    alive[failures] = False

    history = [x.copy()]
    for k in range(steps):
        x = consensus_step(x, A, alive, eps=eps,
                           noise_std=noise_std, p_drop=p_drop)
        history.append(x.copy())

    history = np.array(history)  # shape: (steps+1, N)
    # Consensus error vs time (alive robots only)
    alive_idx = np.where(alive)[0]
    if alive_idx.size == 0:
        return history, None

    mean_traj = history[:, alive_idx].mean(axis=1)
    err_traj = np.linalg.norm(
        history[:, alive_idx] - mean_traj[:, None],
        ord=2, axis=1
    ) / np.sqrt(alive_idx.size)

    return history, err_traj

if __name__ == "__main__":
    for N in [20, 50, 100]:
        _, err = simulate_swarm(N=N, p_fail=0.2, p_drop=0.1)
        if err is not None:
            print(f"N={N}, final error={err[-1]:.4f}")
      
