# Chapter11_Lesson4.py
# Structural controllability graph test for xdot = A x + B u
# The pattern matrices Abar and Bbar use 1 for a free nonzero structural entry
# and 0 for a fixed zero entry.
#
# Test used in this lesson:
#   1. every state vertex is reachable from at least one input vertex;
#   2. the bipartite graph of [Abar Bbar] has a matching that covers all
#      state-row vertices.
#
# This implementation is intentionally written from scratch. For larger systems,
# Python users can also use networkx for graph traversal/matching and python-control
# for numerical state-space analysis.

from collections import deque
from typing import Dict, List, Tuple


def _validate_pattern(Abar: List[List[int]], Bbar: List[List[int]]) -> Tuple[int, int]:
    n = len(Abar)
    if n == 0:
        raise ValueError("Abar must be nonempty.")
    if any(len(row) != n for row in Abar):
        raise ValueError("Abar must be square.")
    if len(Bbar) != n:
        raise ValueError("Bbar must have the same number of rows as Abar.")
    m = len(Bbar[0])
    if m == 0:
        raise ValueError("Bbar must have at least one input column.")
    if any(len(row) != m for row in Bbar):
        raise ValueError("All rows of Bbar must have the same length.")
    return n, m


def _input_reachable_states(Abar: List[List[int]], Bbar: List[List[int]]) -> List[bool]:
    n, m = _validate_pattern(Abar, Bbar)

    # Vertices 0,...,n-1 are states x1,...,xn.
    # Vertices n,...,n+m-1 are inputs u1,...,um.
    adj = [[] for _ in range(n + m)]

    # Abar[i][j] means x_j influences xdot_i, hence x_j -> x_i.
    for i in range(n):
        for j in range(n):
            if Abar[i][j] != 0:
                adj[j].append(i)

    # Bbar[i][k] means u_k influences xdot_i, hence u_k -> x_i.
    for i in range(n):
        for k in range(m):
            if Bbar[i][k] != 0:
                adj[n + k].append(i)

    q = deque(range(n, n + m))
    seen = [False] * (n + m)
    for v in q:
        seen[v] = True

    while q:
        v = q.popleft()
        for w in adj[v]:
            if not seen[w]:
                seen[w] = True
                q.append(w)

    return seen[:n]


def _maximum_bipartite_matching_size(Abar: List[List[int]], Bbar: List[List[int]]) -> int:
    n, m = _validate_pattern(Abar, Bbar)
    left_count = n + m
    right_count = n

    # Left vertices: state columns x1,...,xn and input columns u1,...,um.
    # Right vertices: state rows x1,...,xn.
    adj_left = [[] for _ in range(left_count)]

    # Dynamic structural edges: left state column x_j -> right state row x_i.
    for i in range(n):
        for j in range(n):
            if Abar[i][j] != 0:
                adj_left[j].append(i)

    # Input structural edges: left input column u_k -> right state row x_i.
    for i in range(n):
        for k in range(m):
            if Bbar[i][k] != 0:
                adj_left[n + k].append(i)

    pair_u = [-1] * left_count
    pair_v = [-1] * right_count
    dist = [0] * left_count

    def bfs() -> bool:
        q = deque()
        found_free_right = False
        for u in range(left_count):
            if pair_u[u] == -1:
                dist[u] = 0
                q.append(u)
            else:
                dist[u] = -1

        while q:
            u = q.popleft()
            for v in adj_left[u]:
                mate = pair_v[v]
                if mate == -1:
                    found_free_right = True
                elif dist[mate] == -1:
                    dist[mate] = dist[u] + 1
                    q.append(mate)
        return found_free_right

    def dfs(u: int) -> bool:
        for v in adj_left[u]:
            mate = pair_v[v]
            if mate == -1 or (dist[mate] == dist[u] + 1 and dfs(mate)):
                pair_u[u] = v
                pair_v[v] = u
                return True
        dist[u] = -1
        return False

    matching = 0
    while bfs():
        for u in range(left_count):
            if pair_u[u] == -1 and dfs(u):
                matching += 1
    return matching


def structural_controllability(Abar: List[List[int]], Bbar: List[List[int]]) -> Dict[str, object]:
    n, _ = _validate_pattern(Abar, Bbar)
    reachable = _input_reachable_states(Abar, Bbar)
    matching_size = _maximum_bipartite_matching_size(Abar, Bbar)

    return {
        "all_states_input_reachable": all(reachable),
        "reachable_state_flags": reachable,
        "maximum_matching_size": matching_size,
        "no_dilation_via_full_row_matching": matching_size == n,
        "structurally_controllable": all(reachable) and matching_size == n,
    }


if __name__ == "__main__":
    # Example 1: u1 -> x1 -> x2 -> x3 is structurally controllable.
    A_chain = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
    ]
    B_chain = [
        [1],
        [0],
        [0],
    ]

    # Example 2: u1 -> x1, x1 -> x2, x1 -> x3 is reachable but has a dilation.
    A_dilation = [
        [0, 0, 0],
        [1, 0, 0],
        [1, 0, 0],
    ]
    B_dilation = [
        [1],
        [0],
        [0],
    ]

    print("Chain example:")
    print(structural_controllability(A_chain, B_chain))

    print("\nDilation example:")
    print(structural_controllability(A_dilation, B_dilation))
