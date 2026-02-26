from collections import deque

# Discrete modes
MODE_TRANSIT = 0    # robot moves without object
MODE_TRANSFER = 1   # robot carries object

W, H = 7, 7  # grid dimensions

FREE = 0
OBST = 1

# Simple environment
grid = [[FREE for _ in range(W)] for _ in range(H)]
grid[3][3] = OBST  # one obstacle cell

start_robot = (0, 0)
start_obj   = (2, 2)
goal_obj    = (6, 6)

class HybridState:
    __slots__ = ("mode", "rx", "ry", "ox", "oy")

    def __init__(self, mode, rx, ry, ox, oy):
        self.mode = mode
        self.rx = rx
        self.ry = ry
        self.ox = ox
        self.oy = oy

    def key(self):
        return (self.mode, self.rx, self.ry, self.ox, self.oy)

    def __repr__(self):
        return f"HybridState(mode={self.mode}, r=({self.rx},{self.ry}), o=({self.ox},{self.oy}))"

def in_bounds(x, y):
    return 0 <= x and x < W and 0 <= y and y < H

def is_free(x, y):
    return in_bounds(x, y) and grid[y][x] == FREE

def neighbors(state):
    """Generate successors of a hybrid state."""
    succ = []

    # 4-connected motion
    for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
        nx = state.rx + dx
        ny = state.ry + dy
        if not is_free(nx, ny):
            continue

        if state.mode == MODE_TRANSIT:
            # object stays where it is
            ns = HybridState(MODE_TRANSIT, nx, ny, state.ox, state.oy)
        else:
            # object moves with robot
            ns = HybridState(MODE_TRANSFER, nx, ny, nx, ny)
        succ.append(("move", ns))

    # pick: transit -> transfer
    if state.mode == MODE_TRANSIT and (state.rx, state.ry) == (state.ox, state.oy):
        ns = HybridState(MODE_TRANSFER, state.rx, state.ry, state.rx, state.ry)
        succ.append(("pick", ns))

    # place: transfer -> transit at goal
    if state.mode == MODE_TRANSFER and (state.rx, state.ry) == goal_obj:
        ns = HybridState(MODE_TRANSIT, state.rx, state.ry, state.rx, state.ry)
        succ.append(("place", ns))

    return succ

def is_goal(state):
    return state.mode == MODE_TRANSIT and (state.ox, state.oy) == goal_obj

def bfs_hybrid():
    start = HybridState(MODE_TRANSIT, start_robot[0], start_robot[1],
                        start_obj[0], start_obj[1])
    frontier = deque([start])
    parent = {start.key(): (None, None)}  # key -> (parent_key, action)

    while frontier:
        s = frontier.popleft()
        if is_goal(s):
            # reconstruct plan
            plan = []
            cur = s.key()
            while parent[cur][0] is not None:
                par, act = parent[cur]
                plan.append((act, cur))
                cur = par
            plan.reverse()
            return s, plan

        for act, ns in neighbors(s):
            k = ns.key()
            if k not in parent:
                parent[k] = (s.key(), act)
                frontier.append(ns)
    return None, []

if __name__ == "__main__":
    goal_state, plan = bfs_hybrid()
    if goal_state is None:
        print("No hybrid plan found.")
    else:
        print("Goal state:", goal_state)
        print("Number of steps:", len(plan))
        for act, key in plan:
            print(act, key)
      
