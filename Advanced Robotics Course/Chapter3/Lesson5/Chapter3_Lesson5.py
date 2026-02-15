import math
import random
from typing import List, Tuple, Optional

Point = Tuple[float, float]

class Node:
    def __init__(self, x: Point, parent: Optional["Node"] = None, g: float = math.inf):
        self.x = x
        self.parent = parent
        self.g = g  # cost-to-come

def euclidean(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def line_collision_free(a: Point, b: Point, obstacles) -> bool:
    """
    Placeholder: check segment a-b against obstacles.
    'obstacles' could be polygons, circles, etc.
    """
    # TODO: implement robust collision checking
    return True

def sample_uniform(bounds: Tuple[float, float, float, float]) -> Point:
    xmin, xmax, ymin, ymax = bounds
    return (random.uniform(xmin, xmax), random.uniform(ymin, ymax))

def sample_informed(start: Point, goal: Point, c_best: float,
                    bounds: Tuple[float, float, float, float]) -> Point:
    """
    Sample inside the prolate ellipse defined by start, goal and c_best.
    If no solution yet (c_best == inf), fall back to uniform sampling.
    """
    if not math.isfinite(c_best):
        return sample_uniform(bounds)

    xs, ys = start
    xg, yg = goal
    d_f = euclidean(start, goal)
    if c_best <= d_f:
        # Numerically degenerate: just return start
        return start

    # Coordinate transform: align major axis with x-axis in local frame.
    cx, cy = (xs + xg) / 2.0, (ys + yg) / 2.0
    theta = math.atan2(yg - ys, xg - xs)
    a = c_best / 2.0
    c = d_f / 2.0
    b = math.sqrt(max(a * a - c * c, 0.0))

    # Sample inside unit ball, then scale
    while True:
        u = random.uniform(-1.0, 1.0)
        v = random.uniform(-1.0, 1.0)
        if u * u + v * v <= 1.0:
            break
    # Stretch to ellipse
    lx, ly = a * u, b * v

    # Rotate back and translate to world coordinates
    cos_th, sin_th = math.cos(theta), math.sin(theta)
    wx = cx + cos_th * lx - sin_th * ly
    wy = cy + sin_th * lx + cos_th * ly

    # Optionally clamp to workspace bounds
    xmin, xmax, ymin, ymax = bounds
    wx = min(max(wx, xmin), xmax)
    wy = min(max(wy, ymin), ymax)
    return (wx, wy)

def nearest(nodes: List[Node], x: Point) -> Node:
    return min(nodes, key=lambda n: euclidean(n.x, x))

def neighbors(nodes: List[Node], x: Point, r: float) -> List[Node]:
    return [n for n in nodes if euclidean(n.x, x) <= r]

def bitstar_planner(start: Point, goal: Point,
                    bounds: Tuple[float, float, float, float],
                    obstacles,
                    n_batches: int = 20,
                    batch_size: int = 100,
                    r0: float = 1.0) -> Optional[Node]:
    """
    Extremely simplified BIT*-like planner with informed sampling.
    """
    root = Node(start, parent=None, g=0.0)
    tree_nodes: List[Node] = [root]
    best_goal: Optional[Node] = None
    c_best = math.inf

    for k in range(n_batches):
        # 1. Sample new batch
        samples: List[Point] = []
        for _ in range(batch_size):
            x_samp = sample_informed(start, goal, c_best, bounds)
            samples.append(x_samp)

        # 2. Define connection radius for this batch
        n_total = len(tree_nodes) + len(samples)
        r_n = r0 * math.pow(math.log(max(n_total, 2)) / n_total, 1.0 / 2.0)

        # 3. Populate edge queue: (f_lower, parent_node, x_sample)
        edge_queue: List[Tuple[float, Node, Point]] = []
        for x in samples:
            # consider neighbors in current tree
            for parent in neighbors(tree_nodes, x, r_n):
                g_parent = parent.g
                c_lb = euclidean(parent.x, x)
                h_x = euclidean(x, goal)
                f_lb = g_parent + c_lb + h_x
                if f_lb < c_best:
                    edge_queue.append((f_lb, parent, x))

        # 4. Process edges in order of optimistic cost
        edge_queue.sort(key=lambda tup: tup[0])
        for f_lb, parent, x in edge_queue:
            if f_lb >= c_best:
                break  # cannot improve current solution

            # Attempt to connect if not already in tree
            if any(euclidean(n.x, x) < 1e-9 for n in tree_nodes):
                continue

            if not line_collision_free(parent.x, x, obstacles):
                continue

            g_new = parent.g + euclidean(parent.x, x)
            new_node = Node(x, parent=parent, g=g_new)
            tree_nodes.append(new_node)

            # Check if we reached goal region (here: distance threshold)
            if euclidean(x, goal) <= r_n and line_collision_free(x, goal, obstacles):
                g_goal = g_new + euclidean(x, goal)
                if g_goal < c_best:
                    goal_node = Node(goal, parent=new_node, g=g_goal)
                    tree_nodes.append(goal_node)
                    best_goal = goal_node
                    c_best = g_goal
                    # continue processing; better solution may exist

    return best_goal  # May be None if no solution
      
