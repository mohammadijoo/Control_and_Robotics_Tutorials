from collections import defaultdict, deque

# Grid definition
W, H = 5, 4  # width and height
obstacles = {(1, 1), (2, 1)}
goals = {(4, 3)}
start = (0, 0)

# Actions: up, down, left, right
ACTIONS = {
    "up":    (0, 1),
    "down":  (0, -1),
    "left":  (-1, 0),
    "right": (1, 0),
}

def in_bounds(x, y):
    return 0 <= x < W and 0 <= y < H

# Build state set and transition relation delta(s, u) as a mapping
states = {(x, y) for x in range(W) for y in range(H)}
safe_states = {s for s in states if s not in obstacles}
goal_states = {s for s in states if s in goals}

# Here we use deterministic transitions; for illustration we keep the type "set" of successors
delta = defaultdict(dict)  # delta[s][u] = set of successors

for (x, y) in states:
    for u, (dx, dy) in ACTIONS.items():
        nx, ny = x + dx, y + dy
        if in_bounds(nx, ny):
            succ = (nx, ny)
        else:
            # Invalid moves: stay in place (could model as nondeterministic alternatives)
            succ = (x, y)
        delta[(x, y)][u] = {succ}

def controllable_pre(X):
    """
    Compute Pre(X) = { s | exists u: delta(s,u) subset of X }.
    """
    pre = set()
    for s in safe_states:
        # Skip states already in X to save some work (optional)
        if s in X:
            pre.add(s)
            continue
        for u, succs in delta[s].items():
            # Ensure all successors are in X (nondeterminism-safe)
            if succs and all(sp in X for sp in succs):
                pre.add(s)
                break
    return pre

def winning_set_and_strategy():
    # Fixed-point iteration
    W_set = safe_states & goal_states  # W^0
    while True:
        pre_W = controllable_pre(W_set)
        new_W = W_set | (safe_states & pre_W)
        if new_W == W_set:
            break
        W_set = new_W

    # Synthesize a (memoryless) strategy on winning states
    strategy = {}
    for s in W_set:
        if s in goal_states:
            # Already at goal; we can choose any safe action that keeps us in W_set
            pass
        for u, succs in delta[s].items():
            if succs and all(sp in W_set for sp in succs):
                strategy[s] = u
                break
    return W_set, strategy

W_star, strat = winning_set_and_strategy()

print("Winning set size:", len(W_star))
print("Is start winning?", start in W_star)
print("Strategy at start:", strat.get(start, None))

# Extract one concrete plan from start by following the memoryless strategy
def extract_plan(start_state, strategy, horizon=50):
    if start_state not in strategy:
        return []
    s = start_state
    path = [s]
    for _ in range(horizon):
        if s in goals:
            break
        u = strategy.get(s)
        if u is None:
            break
        succs = delta[s][u]
        # Deterministic choice
        s = next(iter(succs))
        path.append(s)
    return path

plan = extract_plan(start, strat, horizon=50)
print("Plan:", plan)
      
