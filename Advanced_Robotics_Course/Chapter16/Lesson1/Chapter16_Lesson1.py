import collections
import math

# Discrete abstraction parameters
T = 0.1              # sampling time
L = 1.0              # corridor half-width
u_max = 0.5          # max velocity magnitude
N_horizon = 10       # steps
dx = 0.1             # state grid spacing
du = 0.5             # discrete control spacing (values in {-u_max, 0, u_max})

# Discrete sets (representative of configuration space cells)
states = [round(x, 2) for x in 
          [i * dx for i in range(int(-L / dx), int(L / dx) + 1)]]
controls = [-u_max, 0.0, u_max]

def successors(x):
    """All next abstract states from x for all discrete controls."""
    succ = []
    for u in controls:
        x_next = x + T * u
        # snap to grid if inside corridor abstraction
        if -L <= x_next <= L:
            x_next_disc = round(dx * round(x_next / dx), 2)
            succ.append((x_next_disc, u))
        else:
            # If we leave the corridor, record an unsafe successor
            succ.append((None, u))
    return succ

def bfs_check(initial_states, max_depth):
    """
    BFS over abstract transition graph up to horizon max_depth.
    Returns True if safe, False if an unsafe state is reachable.
    """
    # state is (x, step)
    queue = collections.deque()
    visited = set()
    for x0 in initial_states:
        queue.append((x0, 0))
        visited.add((x0, 0))

    while queue:
        x, step = queue.popleft()
        if step == max_depth:
            continue
        for x_next, u in successors(x):
            if x_next is None:
                print(f"Unsafe successor from x={x} with u={u}")
                return False
            node = (x_next, step + 1)
            if node not in visited:
                visited.add(node)
                queue.append(node)
    return True

# Choose initial set strictly inside corridor
initial_states = [0.0]  # or a small neighborhood around 0
is_safe = bfs_check(initial_states, N_horizon)

print("Abstraction declared SAFE up to horizon", N_horizon, ":", is_safe)
      
