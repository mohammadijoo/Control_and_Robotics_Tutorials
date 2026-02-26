import numpy as np
from dataclasses import dataclass

np.random.seed(0)

@dataclass
class Node:
    x: float
    y: float
    parent: int
    cost: float

class RRTStar:
    def __init__(self,
                 x_lim,
                 y_lim,
                 start,
                 goal,
                 step_size=0.1,
                 goal_radius=0.2,
                 max_iter=5000):
        self.x_min, self.x_max = x_lim
        self.y_min, self.y_max = y_lim
        self.start = Node(start[0], start[1], parent=-1, cost=0.0)
        self.goal = np.array(goal, dtype=float)
        self.step_size = step_size
        self.goal_radius = goal_radius
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.obstacles = []  # list of (xmin, ymin, xmax, ymax)

    def add_obstacle(self, rect):
        # rect = (xmin, ymin, xmax, ymax)
        self.obstacles.append(rect)

    def sample_free(self):
        return np.array([
            np.random.uniform(self.x_min, self.x_max),
            np.random.uniform(self.y_min, self.y_max)
        ])

    def dist(self, p, q):
        return np.linalg.norm(p - q)

    def nearest(self, x_rand):
        dists = [self.dist(np.array([n.x, n.y]), x_rand) for n in self.nodes]
        return int(np.argmin(dists))

    def steer(self, x_nearest, x_rand):
        direction = x_rand - x_nearest
        norm = np.linalg.norm(direction)
        if norm <= self.step_size:
            return x_rand
        return x_nearest + self.step_size * direction / norm

    def line_collision_free(self, p, q, resolution=10):
        # simple discrete collision check along segment
        for i in range(resolution + 1):
            alpha = i / resolution
            x = (1 - alpha) * p + alpha * q
            if self.in_obstacle(x):
                return False
        return True

    def in_obstacle(self, x):
        px, py = x
        for (xmin, ymin, xmax, ymax) in self.obstacles:
            if xmin <= px <= xmax and ymin <= py <= ymax:
                return True
        return False

    def near(self, x_new, gamma_rrt=1.0):
        n = len(self.nodes)
        d = 2  # dimension
        if n == 1:
            return [0]
        r_n = min(
            gamma_rrt * (np.log(n) / n)**(1.0 / d),
            self.step_size * 10.0
        )
        idxs = []
        for i, node in enumerate(self.nodes):
            p = np.array([node.x, node.y])
            if self.dist(p, x_new) <= r_n:
                idxs.append(i)
        return idxs

    def cost_to(self, idx):
        return self.nodes[idx].cost

    def plan(self):
        goal_idx = None
        for k in range(self.max_iter):
            x_rand = self.sample_free()
            idx_nearest = self.nearest(x_rand)
            x_near = np.array([self.nodes[idx_nearest].x,
                               self.nodes[idx_nearest].y])
            x_new = self.steer(x_near, x_rand)
            if self.in_obstacle(x_new):
                continue
            if not self.line_collision_free(x_near, x_new):
                continue

            neighbor_idxs = self.near(x_new)
            # choose best parent
            best_parent = idx_nearest
            best_cost = self.cost_to(idx_nearest) + self.dist(x_near, x_new)
            for i in neighbor_idxs:
                xi = np.array([self.nodes[i].x, self.nodes[i].y])
                if self.line_collision_free(xi, x_new):
                    cand_cost = self.cost_to(i) + self.dist(xi, x_new)
                    if cand_cost < best_cost:
                        best_parent = i
                        best_cost = cand_cost

            new_node = Node(x_new[0], x_new[1], parent=best_parent,
                            cost=best_cost)
            self.nodes.append(new_node)
            new_idx = len(self.nodes) - 1

            # rewire neighbors
            for i in neighbor_idxs:
                if i == best_parent:
                    continue
                xi = np.array([self.nodes[i].x, self.nodes[i].y])
                if not self.line_collision_free(x_new, xi):
                    continue
                cand_cost = best_cost + self.dist(x_new, xi)
                if cand_cost < self.nodes[i].cost:
                    self.nodes[i].parent = new_idx
                    self.nodes[i].cost = cand_cost

            # check if we reached goal region
            if self.dist(x_new, self.goal) <= self.goal_radius:
                if (goal_idx is None or
                    best_cost + self.dist(x_new, self.goal) <
                    self.nodes[goal_idx].cost):
                    goal_idx = new_idx

        return self.extract_path(goal_idx)

    def extract_path(self, goal_idx):
        if goal_idx is None:
            return None
        path = [self.goal.copy()]
        idx = goal_idx
        while idx != -1:
            n = self.nodes[idx]
            path.append(np.array([n.x, n.y]))
            idx = n.parent
        path.reverse()
        return np.array(path)

# Example usage
if __name__ == "__main__":
    planner = RRTStar(x_lim=(0.0, 1.0),
                      y_lim=(0.0, 1.0),
                      start=(0.1, 0.1),
                      goal=(0.9, 0.9))
    planner.add_obstacle((0.3, 0.3, 0.7, 0.4))
    path = planner.plan()
    print("Path:", path)
      
