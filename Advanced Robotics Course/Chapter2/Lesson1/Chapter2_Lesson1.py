from collections import deque
from typing import Dict, List, Hashable, Optional, Tuple

# Directed graph with nonnegative weights
Adjacency = Dict[Hashable, List[Tuple[Hashable, float]]]

def bfs_unweighted(adj: Dict[Hashable, List[Hashable]],
                   start,
                   goal) -> Optional[List[Hashable]]:
    """
    Breadth-first search for unweighted graphs.
    Returns a path with the minimal number of edges, or None if unreachable.
    """
    queue = deque([start])
    parent = {start: None}

    while queue:
        v = queue.popleft()
        if v == goal:
            break
        for w in adj.get(v, []):
            if w not in parent:
                parent[w] = v
                queue.append(w)

    if goal not in parent:
        return None

    # Reconstruct path
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    path.reverse()
    return path

def dijkstra(adj: Adjacency,
             start,
             goal) -> Optional[Tuple[float, List[Hashable]]]:
    """
    Dijkstra's algorithm for nonnegative-weight directed graphs.
    Returns (cost, path) or None if goal unreachable.
    """
    import heapq

    dist = {start: 0.0}
    parent: Dict[Hashable, Optional[Hashable]] = {start: None}
    heap = [(0.0, start)]

    while heap:
        cost_v, v = heapq.heappop(heap)
        if v == goal:
            break
        if cost_v > dist[v]:
            continue
        for w, w_cost in adj.get(v, []):
            new_cost = cost_v + w_cost
            if w not in dist or new_cost < dist[w]:
                dist[w] = new_cost
                parent[w] = v
                heapq.heappush(heap, (new_cost, w))

    if goal not in dist:
        return None

    # Reconstruct path
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    path.reverse()
    return dist[goal], path

# Example: 2-DOF grid in joint space
def make_joint_grid_graph(q1_vals, q2_vals, step_cost=1.0) -> Adjacency:
    """
    Build a simple 4-connected grid in joint space (q1, q2).
    Here we ignore collisions; in practice insert collision checks.
    """
    adj: Adjacency = {}
    for q1 in q1_vals:
        for q2 in q2_vals:
            v = (q1, q2)
            nbrs = []
            # 4-connected neighbors
            for dq1, dq2 in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nq1 = q1 + dq1
                nq2 = q2 + dq2
                if nq1 in q1_vals and nq2 in q2_vals:
                    nbrs.append(((nq1, nq2), step_cost))
            adj[v] = nbrs
    return adj

if __name__ == "__main__":
    q_values = list(range(-2, 3))
    graph = make_joint_grid_graph(q_values, q_values)
    start = (0, 0)
    goal = (2, 2)
    cost, path = dijkstra(graph, start, goal)
    print("Optimal cost:", cost)
    print("Path:", path)
      
