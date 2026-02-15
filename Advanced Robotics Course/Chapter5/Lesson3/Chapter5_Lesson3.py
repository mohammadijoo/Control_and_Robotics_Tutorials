import numpy as np

class Node:
    def __init__(self, state, parent=None, cost=0.0):
        self.state = np.array(state, dtype=float)
        self.parent = parent
        self.cost = float(cost)

def distance(x1, x2, w_pos=1.0, w_vel=0.1):
    # x = [x, y, vx, vy]
    dp = x1[:2] - x2[:2]
    dv = x1[2:] - x2[2:]
    return np.sqrt(w_pos * np.dot(dp, dp) + w_vel * np.dot(dv, dv))

def forward_propagate(x0, u, dt, steps):
    """Explicit Euler integration for double integrator."""
    x = np.array(x0, dtype=float)
    cost = 0.0
    traj = [x.copy()]
    for _ in range(steps):
        x[0] += dt * x[2]
        x[1] += dt * x[3]
        x[2] += dt * u[0]
        x[3] += dt * u[1]
        cost += dt  # time cost
        traj.append(x.copy())
    return x, cost, np.array(traj)

def collision_free(traj, obstacles):
    # obstacles: list of (cx, cy, r)
    for x in traj:
        for (cx, cy, r) in obstacles:
            if (x[0] - cx) ** 2 + (x[1] - cy) ** 2 <= r ** 2:
                return False
    return True

def find_nearest(tree, x_rand):
    d_min = float("inf")
    best = None
    for node in tree:
        d = distance(node.state, x_rand)
        if d < d_min:
            d_min = d
            best = node
    return best

def find_neighbors(tree, x_new, r_n):
    return [node for node in tree if distance(node.state, x_new) <= r_n]

def kinodynamic_rrt_star(x_init, goal_region, bounds, obstacles,
                         u_set, dt=0.1, steps=5, n_iter=500,
                         gamma=30.0):
    """
    x_init: initial state [x,y,vx,vy]
    goal_region: lambda x -> bool
    bounds: [xmin, xmax, ymin, ymax, vmax]
    u_set: list of candidate accelerations [(ux, uy), ...]
    """
    dim = 4
    X_free_measure = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2])
    tree = [Node(x_init)]
    best_goal = None

    for n in range(1, n_iter + 1):
        # sample state (velocity bounded)
        x_rand = np.array([
            np.random.uniform(bounds[0], bounds[1]),
            np.random.uniform(bounds[2], bounds[3]),
            np.random.uniform(-bounds[4], bounds[4]),
            np.random.uniform(-bounds[4], bounds[4])
        ])

        x_near_node = find_nearest(tree, x_rand)
        x_near = x_near_node.state

        # choose best control among discrete set
        best_x_new, best_traj = None, None
        best_edge_cost = float("inf")
        for u in u_set:
            x_new, edge_cost, traj = forward_propagate(x_near, u, dt, steps)
            if not (bounds[0] <= x_new[0] <= bounds[1] and
                    bounds[2] <= x_new[1] <= bounds[3]):
                continue
            if not collision_free(traj, obstacles):
                continue
            if edge_cost < best_edge_cost:
                best_edge_cost = edge_cost
                best_x_new = x_new
                best_traj = traj

        if best_x_new is None:
            continue

        r_n = min(
            (gamma * (np.log(n) / n)) ** (1.0 / dim),
            max(bounds[1] - bounds[0], bounds[3] - bounds[2])
        )

        neighbors = find_neighbors(tree, best_x_new, r_n)
        # choose parent that minimizes cost-to-come
        x_new_parent = x_near_node
        new_cost = x_near_node.cost + best_edge_cost

        for node in neighbors:
            # approximate local propagation cost via distance as heuristic
            cand_x_new, cand_cost, cand_traj = forward_propagate(
                node.state, u_set[0], dt, steps
            )
            # for simplicity, assume same cost as best_edge_cost if collision-free
            if collision_free(cand_traj, obstacles):
                cand_total = node.cost + best_edge_cost
                if cand_total < new_cost:
                    new_cost = cand_total
                    x_new_parent = node

        new_node = Node(best_x_new, parent=x_new_parent, cost=new_cost)
        tree.append(new_node)

        # rewiring step
        for node in neighbors:
            # naive local cost estimate (reuse best_edge_cost)
            if new_node.cost + best_edge_cost < node.cost:
                # check collision-free local edge (skipped detail here)
                node.parent = new_node
                node.cost = new_node.cost + best_edge_cost

        if goal_region(new_node.state):
            if best_goal is None or new_node.cost < best_goal.cost:
                best_goal = new_node

    return tree, best_goal
      
