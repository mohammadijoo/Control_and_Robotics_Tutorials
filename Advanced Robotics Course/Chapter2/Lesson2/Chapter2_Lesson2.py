import heapq
from typing import Dict, Tuple, Callable, Hashable, Iterable

Node = Hashable
Graph = Dict[Node, Dict[Node, float]]  # adjacency: g[u][v] = cost(u,v)

def astar(
    graph: Graph,
    start: Node,
    goal: Node,
    heuristic: Callable[[Node], float]
) -> Tuple[float, Tuple[Node, ...]]:
    """A* search on a directed weighted graph."""
    open_heap = []  # (f, g, node)
    heapq.heappush(open_heap, (heuristic(start), 0.0, start))
    g_cost = {start: 0.0}
    parent: Dict[Node, Node] = {}

    closed = set()

    while open_heap:
        f, g, v = heapq.heappop(open_heap)
        if v in closed:
            continue
        if v == goal:
            # reconstruct path
            path = [v]
            while v in parent:
                v = parent[v]
                path.append(v)
            path.reverse()
            return g, tuple(path)

        closed.add(v)
        for u, c in graph.get(v, {}).items():
            if c < 0:
                raise ValueError("Negative edge costs are not allowed for A*.")
            g_tent = g + c
            if u not in g_cost or g_tent < g_cost[u]:
                g_cost[u] = g_tent
                parent[u] = v
                f_u = g_tent + heuristic(u)
                heapq.heappush(open_heap, (f_u, g_tent, u))

    raise RuntimeError("No path found from start to goal.")

# Example heuristic for a 2D grid C-space (e.g., planar mobile base):
def manhattan_heuristic(goal_xy):
    gx, gy = goal_xy
    def h(node_xy):
        x, y = node_xy
        return abs(x - gx) + abs(y - gy)
    return h
      
