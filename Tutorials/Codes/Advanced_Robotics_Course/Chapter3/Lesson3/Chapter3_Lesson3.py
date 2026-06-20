import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple

# 2D configuration
Config = np.ndarray  # shape (2,)

@dataclass
class Node:
    q: Config
    parent: Optional[int]  # index of parent node in the tree list

@dataclass
class Tree:
    nodes: List[Node]

    def add_node(self, q: Config, parent_idx: Optional[int]) -> int:
        self.nodes.append(Node(q=q, parent=parent_idx))
        return len(self.nodes) - 1

    def nearest(self, q: Config) -> int:
        dists = [np.linalg.norm(n.q - q) for n in self.nodes]
        return int(np.argmin(dists))

def steer(q_near: Config, q_rand: Config, eta: float) -> Config:
    direction = q_rand - q_near
    dist = np.linalg.norm(direction)
    if dist <= eta:
        return q_rand
    return q_near + eta * direction / dist

def collision_free(q1: Config, q2: Config, obstacles) -> bool:
    """
    obstacles: list of axis-aligned rectangles [xmin, xmax, ymin, ymax]
    Simple discrete check; for real robots, integrate with FCL / MoveIt etc.
    """
    n_steps = 20
    for i in range(n_steps + 1):
        alpha = i / n_steps
        q = (1 - alpha) * q1 + alpha * q2
        x, y = q
        for (xmin, xmax, ymin, ymax) in obstacles:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return False
    return True

def sample_config(bounds: Tuple[Tuple[float,float], Tuple[float,float]]) -> Config:
    (xmin, xmax), (ymin, ymax) = bounds
    return np.array([
        np.random.uniform(xmin, xmax),
        np.random.uniform(ymin, ymax)
    ])

def build_rrt(q_start: Config,
              q_goal: Config,
              bounds,
              obstacles,
              eta: float = 0.2,
              goal_radius: float = 0.1,
              max_iter: int = 5000) -> Optional[List[Config]]:
    tree = Tree(nodes=[])
    tree.add_node(q_start, parent_idx=None)

    for k in range(max_iter):
        # Goal bias: occasionally sample the goal directly
        if np.random.rand() < 0.1:
            q_rand = q_goal
        else:
            q_rand = sample_config(bounds)

        idx_near = tree.nearest(q_rand)
        q_near = tree.nodes[idx_near].q
        q_new = steer(q_near, q_rand, eta)

        if collision_free(q_near, q_new, obstacles):
            idx_new = tree.add_node(q_new, parent_idx=idx_near)

            if np.linalg.norm(q_new - q_goal) <= goal_radius:
                # Reconstruct path
                path = []
                cur = idx_new
                while cur is not None:
                    path.append(tree.nodes[cur].q)
                    cur = tree.nodes[cur].parent
                path.reverse()
                return path
    return None

def extend(tree: Tree, q_rand: Config, eta: float, obstacles) -> Tuple[str, Optional[int]]:
    """
    RRT-Connect style extend: single step toward q_rand.
    Returns (status, new_index).
    """
    idx_near = tree.nearest(q_rand)
    q_near = tree.nodes[idx_near].q
    q_new = steer(q_near, q_rand, eta)
    if not collision_free(q_near, q_new, obstacles):
        return "Trapped", None
    idx_new = tree.add_node(q_new, idx_near)
    if np.allclose(q_new, q_rand):
        return "Reached", idx_new
    return "Advanced", idx_new

def connect(tree: Tree, q_target: Config, eta: float, obstacles) -> Tuple[str, Optional[int]]:
    """
    Aggressively extend tree toward q_target until trapped or reached.
    """
    status = "Advanced"
    last_idx = None
    while status == "Advanced":
        status, last_idx = extend(tree, q_target, eta, obstacles)
    return status, last_idx

# Example usage (2D point robot):
if __name__ == "__main__":
    np.random.seed(0)
    q_start = np.array([0.1, 0.1])
    q_goal = np.array([0.9, 0.9])
    bounds = ((0.0, 1.0), (0.0, 1.0))
    obstacles = [
        (0.3, 0.7, 0.4, 0.6),  # rectangular obstacle
    ]
    path = build_rrt(q_start, q_goal, bounds, obstacles)
    if path is None:
        print("No path found")
    else:
        print("Path length:", len(path))
      
