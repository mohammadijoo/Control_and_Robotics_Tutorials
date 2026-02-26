import numpy as np

class Robot:
    def __init__(self, idx, values, epsilon=0.01):
        self.idx = idx
        self.values = np.array(values, dtype=float)  # v_ij
        self.epsilon = epsilon
        self.assigned_task = None

    def is_unassigned(self):
        return self.assigned_task is None

    def compute_bid(self, prices):
        # reduced utility u_j = v_ij - p_j
        reduced = self.values - prices
        # best and second-best tasks
        j1 = int(np.argmax(reduced))
        best = reduced[j1]
        # mask out j1 to get second-best
        mask = np.ones_like(reduced, dtype=bool)
        mask[j1] = False
        if np.any(mask):
            second = np.max(reduced[mask])
        else:
            second = -np.inf
        bid = prices[j1] + (best - second) + self.epsilon
        return j1, bid

def distributed_auction(value_matrix, epsilon=0.01, max_iters=1000):
    """
    value_matrix: shape (n_robots, n_tasks), v_ij >= 0
    """
    n_robots, n_tasks = value_matrix.shape
    robots = [Robot(i, value_matrix[i], epsilon) for i in range(n_robots)]
    prices = np.zeros(n_tasks)
    winners = np.full(n_tasks, fill_value=-1, dtype=int)

    it = 0
    while it < max_iters and any(r.is_unassigned() for r in robots):
        it += 1
        # local bids
        bids_for_task = {j: [] for j in range(n_tasks)}
        for r in robots:
            if r.is_unassigned():
                j, b = r.compute_bid(prices)
                bids_for_task[j].append((b, r.idx))

        # "distributed" resolution: for each task, keep highest bid
        for j, bids in bids_for_task.items():
            if not bids:
                continue
            bids.sort(key=lambda x: x[0], reverse=True)
            best_bid, best_robot = bids[0]
            prices[j] = best_bid
            # unassign previous winner if any
            prev = winners[j]
            if prev != -1 and prev != best_robot:
                robots[prev].assigned_task = None
            winners[j] = best_robot
            robots[best_robot].assigned_task = j

    assignment = np.full(n_robots, fill_value=-1, dtype=int)
    for j, r_idx in enumerate(winners):
        if r_idx != -1:
            assignment[r_idx] = j

    return assignment, prices

if __name__ == "__main__":
    # Example: 3 robots, 3 tasks, valuations inversely related to travel cost
    value_matrix = np.array([
        [10.0,  8.0,  9.0],
        [ 9.0, 11.0,  7.0],
        [ 8.5,  9.5, 12.0],
    ])
    assignment, prices = distributed_auction(value_matrix, epsilon=0.01)
    print("Assignment (robot -> task):", assignment)
    print("Final prices:", prices)
      
