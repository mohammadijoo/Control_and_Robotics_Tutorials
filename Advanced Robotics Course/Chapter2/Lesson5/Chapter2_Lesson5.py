import heapq
import math
from typing import Tuple, List, Dict

# 7-DOF joint limits (radians)
JOINT_LIMITS = [
    (-2.9, 2.9),
    (-2.0, 2.0),
    (-2.0, 2.0),
    (-2.9, 2.9),
    (-2.0, 2.0),
    (-2.0, 2.0),
    (-3.1, 3.1),
]

RESOLUTION = [0.1] * 7  # joint discretization

def discretize(q: List[float]) -> Tuple[int, ...]:
    idx = []
    for i, (q_min, q_max) in enumerate(JOINT_LIMITS):
        delta = RESOLUTION[i]
        k = int(round((q[i] - q_min) / delta))
        # clamp to grid
        max_k = int(round((q_max - q_min) / delta))
        k = max(0, min(k, max_k))
        idx.append(k)
    return tuple(idx)

def undisc(k: Tuple[int, ...]) -> List[float]:
    q = []
    for i, ki in enumerate(k):
        q_min, q_max = JOINT_LIMITS[i]
        delta = RESOLUTION[i]
        q.append(q_min + ki * delta)
    return q

def neighbors(k: Tuple[int, ...]) -> List[Tuple[int, ...]]:
    # single-joint +/- 1-step primitives
    nbrs = []
    for i in range(len(k)):
        for step in (-1, 1):
            kk = list(k)
            kk[i] += step
            # bounds in index space
            q_min, q_max = JOINT_LIMITS[i]
            delta = RESOLUTION[i]
            max_k = int(round((q_max - q_min) / delta))
            if 0 <= kk[i] <= max_k:
                q = undisc(kk)
                if is_collision_free(q):  # hook for robotics middleware
                    nbrs.append(tuple(kk))
    return nbrs

def edge_cost(k1: Tuple[int, ...], k2: Tuple[int, ...]) -> float:
    q1 = undisc(k1)
    q2 = undisc(k2)
    # weighted L1 distance
    w = [1.0] * len(q1)
    return sum(w[i] * abs(q2[i] - q1[i]) for i in range(len(q1)))

def h_joint(k: Tuple[int, ...], goal_k: Tuple[int, ...]) -> float:
    q = undisc(k)
    qg = undisc(goal_k)
    # Euclidean joint-space heuristic (admissible if edge_cost dominates it)
    return math.sqrt(sum((qi - qgi) ** 2 for qi, qgi in zip(q, qg)))

def is_collision_free(q: List[float]) -> bool:
    # Placeholder: in a real system, call MoveIt/OMPL or a physics engine.
    # Here we pretend the whole space is free.
    return True

def astar(start_q: List[float], goal_q: List[float]) -> List[List[float]]:
    start_k = discretize(start_q)
    goal_k = discretize(goal_q)

    open_heap: List[Tuple[float, Tuple[int, ...]]] = []
    heapq.heappush(open_heap, (0.0, start_k))

    g_score: Dict[Tuple[int, ...], float] = {start_k: 0.0}
    parent: Dict[Tuple[int, ...], Tuple[int, ...]] = {}

    while open_heap:
        f, current = heapq.heappop(open_heap)
        if current == goal_k:
            # reconstruct path
            path_k = [current]
            while current in parent:
                current = parent[current]
                path_k.append(current)
            path_k.reverse()
            return [undisc(k) for k in path_k]

        for nb in neighbors(current):
            tentative_g = g_score[current] + edge_cost(current, nb)
            if tentative_g < g_score.get(nb, float("inf")):
                g_score[nb] = tentative_g
                parent[nb] = current
                f_nb = tentative_g + h_joint(nb, goal_k)
                heapq.heappush(open_heap, (f_nb, nb))

    raise RuntimeError("No path found")

if __name__ == "__main__":
    q_start = [0.0, -1.0, 0.5, 0.0, 0.0, 0.5, 0.0]
    q_goal  = [1.0,  0.5, 0.0, 0.5, 0.0, 1.0, 0.0]
    path = astar(q_start, q_goal)
    print("Path length (states):", len(path))
      
