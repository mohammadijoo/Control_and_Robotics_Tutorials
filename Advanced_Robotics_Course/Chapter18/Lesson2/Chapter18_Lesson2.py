import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

np.random.seed(0)

@dataclass
class PlanningTask2D:
    start: np.ndarray        # shape (2,)
    goal: np.ndarray         # shape (2,)
    obstacles: np.ndarray    # shape (K, 4), axis-aligned boxes [x_min, y_min, x_max, y_max]

def random_box():
    x1, y1 = np.random.rand(2)
    x2, y2 = np.random.rand(2)
    x_min, x_max = sorted([x1, x2])
    y_min, y_max = sorted([y1, y2])
    # Shrink to avoid full blockage
    w = (x_max - x_min) * 0.7
    h = (y_max - y_min) * 0.7
    return np.array([x_min, y_min, x_min + w, y_min + h])

def sample_task(num_obstacles: int) -> PlanningTask2D:
    start = np.random.rand(2)
    goal = np.random.rand(2)
    obstacles = np.stack([random_box() for _ in range(num_obstacles)], axis=0)
    return PlanningTask2D(start=start, goal=goal, obstacles=obstacles)

def clearance_feature(task: PlanningTask2D, num_samples: int = 32) -> float:
    """
    Crude estimate of average clearance along straight-line path.
    """
    ts = np.linspace(0.0, 1.0, num_samples)
    pts = (1 - ts)[:, None] * task.start[None, :] + ts[:, None] * task.goal[None, :]
    min_dists = []
    for p in pts:
        # signed distance to axis-aligned boxes (positive outside)
        d = []
        for (xmin, ymin, xmax, ymax) in task.obstacles:
            dx = max(xmin - p[0], 0, p[0] - xmax)
            dy = max(ymin - p[1], 0, p[1] - ymax)
            d.append(np.hypot(dx, dy))
        min_dists.append(min(d) if d else 1.0)
    return float(np.mean(min_dists))

def feature_vector(task: PlanningTask2D) -> np.ndarray:
    # Example features: num_obstacles, path_length, avg_clearance
    num_obs = float(task.obstacles.shape[0])
    path_len = float(np.linalg.norm(task.goal - task.start))
    clear = clearance_feature(task)
    return np.array([num_obs, path_len, clear], dtype=float)

def greedy_k_center(tasks: List[PlanningTask2D], B: int) -> List[int]:
    """
    Greedy k-center over feature vectors. Returns indices of selected tasks.
    """
    feats = np.stack([feature_vector(t) for t in tasks], axis=0)
    # Normalize features for fair distances
    mu = feats.mean(axis=0, keepdims=True)
    sigma = feats.std(axis=0, keepdims=True) + 1e-8
    feats_n = (feats - mu) / sigma

    n = feats_n.shape[0]
    # Start with a random center
    centers = [np.random.randint(0, n)]
    # Precompute distance matrix (small demo)
    dists = np.linalg.norm(feats_n[:, None, :] - feats_n[None, :, :], axis=-1)

    while len(centers) < B:
        # For each candidate, compute distance to closest chosen center
        min_to_center = dists[:, centers].min(axis=1)
        # Pick point with maximum such distance
        next_center = int(np.argmax(min_to_center))
        centers.append(next_center)
    return centers

# Generate a pool of tasks
pool_size = 200
tasks = []
for i in range(pool_size):
    # mixture of sparse and cluttered scenes
    num_obs = np.random.choice([2, 4, 8, 12], p=[0.2, 0.3, 0.3, 0.2])
    tasks.append(sample_task(num_obs))

# Select B benchmark tasks
B = 40
selected_indices = greedy_k_center(tasks, B)
benchmark_tasks = [tasks[i] for i in selected_indices]

print(f"Selected {len(benchmark_tasks)} tasks for the benchmark suite.")
      
