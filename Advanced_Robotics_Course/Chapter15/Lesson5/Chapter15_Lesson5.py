import numpy as np
import matplotlib.pyplot as plt

def initialize_positions(N, box_size=10.0, seed=0):
    rng = np.random.default_rng(seed)
    # Positions: shape (N, 2)
    return box_size * (rng.random((N, 2)) - 0.5)

def compute_neighbors(positions, R):
    N = positions.shape[0]
    neighbors = [[] for _ in range(N)]
    R2 = R * R
    for i in range(N):
        for j in range(i + 1, N):
            d = positions[j] - positions[i]
            dist2 = np.dot(d, d)
            if dist2 <= R2:
                neighbors[i].append(j)
                neighbors[j].append(i)
    return neighbors

def step_swarm(positions, h, R, w_att, w_rep, d_min, v_max):
    N = positions.shape[0]
    neighbors = compute_neighbors(positions, R)
    vel = np.zeros_like(positions)

    for i in range(N):
        p_i = positions[i]
        att = np.zeros(2)
        rep = np.zeros(2)
        for j in neighbors[i]:
            p_j = positions[j]
            diff = p_j - p_i
            att += diff
            # Short-range repulsion
            dist2 = np.dot(diff, diff)
            if dist2 > 1e-12:
                dist = np.sqrt(dist2)
                if dist <= d_min:
                    rep += -diff / (dist**3)
        u_i = w_att * att + w_rep * rep
        vel[i] = u_i

    # Saturate velocities
    speeds = np.linalg.norm(vel, axis=1)
    mask = speeds > v_max
    vel[mask] *= (v_max / speeds[mask])[:, None]

    # Euler integration
    new_positions = positions + h * vel
    return new_positions

def run_simulation(N=50, steps=500, h=0.05, R=2.0,
                   w_att=0.2, w_rep=0.05, d_min=0.5, v_max=0.5):
    positions = initialize_positions(N)
    traj = [positions.copy()]

    for k in range(steps):
        positions = step_swarm(positions, h, R, w_att, w_rep, d_min, v_max)
        traj.append(positions.copy())
    return np.array(traj)  # shape (steps+1, N, 2)

def plot_trajectories(traj, stride=10):
    steps, N, _ = traj.shape
    plt.figure()
    for i in range(N):
        plt.plot(traj[::stride, i, 0], traj[::stride, i, 1], alpha=0.6)
        plt.scatter(traj[-1, i, 0], traj[-1, i, 1], s=10)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Swarm trajectories (subsampled)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    traj = run_simulation()
    plot_trajectories(traj)
      
