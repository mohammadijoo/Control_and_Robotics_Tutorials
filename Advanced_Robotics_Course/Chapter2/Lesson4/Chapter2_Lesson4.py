import heapq
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple, Set

State = Any
Cost = float

def astar(
    start: State,
    is_goal: Callable[[State], bool],
    successors: Callable[[State], Iterable[Tuple[State, Cost]]],
    heuristic: Callable[[State], Cost],
) -> Optional[List[State]]:
    """
    A* search with admissible heuristic.

    Args:
        start: initial state (node in the graph).
        is_goal: predicate to test whether a state is goal.
        successors: function mapping state -> iterable of (next_state, edge_cost).
        heuristic: admissible heuristic h(x) estimating remaining cost.

    Returns:
        A list of states from start to a goal if found, otherwise None.
    """
    # g-values: known best cost to reach each state
    g: Dict[State, Cost] = {start: 0.0}
    parent: Dict[State, Optional[State]] = {start: None}

    counter = 0  # tie-breaker to keep heapq stable
    open_heap: List[Tuple[Cost, int, State]] = []
    heapq.heappush(open_heap, (heuristic(start), counter, start))

    closed: Set[State] = set()

    while open_heap:
        f, _, s = heapq.heappop(open_heap)
        if s in closed:
            continue
        closed.add(s)

        if is_goal(s):
            # reconstruct path
            path: List[State] = []
            cur: Optional[State] = s
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            path.reverse()
            return path

        g_s = g[s]
        for s_next, c in successors(s):
            if c < 0.0:
                raise ValueError("Edge cost must be non-negative for A* guarantees.")
            new_g = g_s + c
            if s_next not in g or new_g < g[s_next]:
                g[s_next] = new_g
                parent[s_next] = s
                counter += 1
                f_next = new_g + heuristic(s_next)
                heapq.heappush(open_heap, (f_next, counter, s_next))

    return None

# Example: 2D grid with 4-connected moves
def grid_successors(pos):
    x, y = pos
    steps = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    for dx, dy in steps:
        nxt = (x + dx, y + dy)
        # Here we could check obstacles or bounds
        yield nxt, 1.0

def manhattan_heuristic(goal):
    gx, gy = goal
    def h(state):
        x, y = state
        return abs(x - gx) + abs(y - gy)
    return h

if __name__ == "__main__":
    start = (0, 0)
    goal = (5, 7)
    h = manhattan_heuristic(goal)
    path = astar(start, lambda s: s == goal, grid_successors, h)
    print("Path:", path)
      
