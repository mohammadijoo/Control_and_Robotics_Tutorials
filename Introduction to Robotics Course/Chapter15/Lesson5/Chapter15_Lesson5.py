import numpy as np

def group_risk(costs, groups, g):
    """
    costs : 1D numpy array of nonnegative costs for each interaction
    groups: 1D numpy array of group labels (0 or 1)
    g     : group of interest (0 or 1)
    """
    mask = (groups == g)
    if not np.any(mask):
        raise ValueError(f"No samples for group {g}")
    return float(costs[mask].mean())

def disparity(costs, groups):
    r0 = group_risk(costs, groups, 0)
    r1 = group_risk(costs, groups, 1)
    return abs(r1 - r0)

# Example usage with a simple threshold
eps = 0.05  # allowed disparity
costs = np.array([0.0, 1.0, 0.2, 0.1, 0.7, 0.3])
groups = np.array([0, 0, 1, 1, 1, 0])

d = disparity(costs, groups)
print("Estimated disparity:", d)
if d > eps:
    print("Warning: fairness constraint violated.")
      
