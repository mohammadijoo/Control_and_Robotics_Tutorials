from collections import deque

# Explicit transition system for a simple mobile base on a grid
# States are integers 0..N-1; edges encode discrete moves.
N = 8
adj = {
    0: [1, 2],
    1: [3],
    2: [3, 4],
    3: [5],
    4: [5, 6],
    5: [7],
    6: [7],
    7: []
}

S0 = {0}       # initial state
Bad = {6}      # e.g., "collision" region

def forward_reach(adj, S0):
    """Compute R* using BFS-like forward reachability."""
    reachable = set(S0)
    queue = deque(S0)
    while queue:
        s = queue.popleft()
        for s_next in adj.get(s, []):
            if s_next not in reachable:
                reachable.add(s_next)
                queue.append(s_next)
    return reachable

def backward_reach(adj, Bad):
    """Compute B* using reverse edges and backward reachability."""
    # Build reverse adjacency
    radj = {s: [] for s in adj}
    for s, outs in adj.items():
        for s_next in outs:
            radj[s_next].append(s)

    backward = set(Bad)
    queue = deque(Bad)
    while queue:
        s = queue.popleft()
        for s_prev in radj.get(s, []):
            if s_prev not in backward:
                backward.add(s_prev)
                queue.append(s_prev)
    return backward

R_star = forward_reach(adj, S0)
B_star = backward_reach(adj, Bad)

print("Reachable states from S0:", R_star)
print("Backward reachable states to Bad:", B_star)
print("Safety holds?" , S0.isdisjoint(B_star))
      
