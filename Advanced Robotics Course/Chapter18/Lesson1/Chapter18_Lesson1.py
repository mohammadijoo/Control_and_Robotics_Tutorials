import numpy as np

def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class ConfigMetric:
    """
    Weighted configuration-space metric with support for revolute joints.

    weights: 1D array of positive weights.
    revolute: boolean mask, True for revolute joints.
    """
    def __init__(self, weights, revolute):
        self.W = np.diag(np.asarray(weights, dtype=float))
        self.revolute = np.asarray(revolute, dtype=bool)

    def distance(self, q1, q2) -> float:
        q1 = np.asarray(q1, dtype=float)
        q2 = np.asarray(q2, dtype=float)
        diff = q2 - q1
        # Wrap differences for revolute joints
        if self.revolute.any():
            diff[self.revolute] = (diff[self.revolute] + np.pi) % (2.0 * np.pi) - np.pi
        return float(np.sqrt(diff.T @ self.W @ diff))

    def path_length(self, path) -> float:
        """
        path: iterable of configurations (list or array of shape (N, n)).
        """
        path = np.asarray(path, dtype=float)
        total = 0.0
        for i in range(len(path) - 1):
            total += self.distance(path[i], path[i + 1])
        return float(total)


def path_clearance(path, dist_to_obstacles) -> float:
    """
    path: iterable of configurations.
    dist_to_obstacles: callable q -> clearance(q).
    """
    return float(min(dist_to_obstacles(q) for q in path))


def composite_cost(path, metric: ConfigMetric, dist_to_obstacles,
                   alpha: float = 1.0, beta: float = 1.0) -> float:
    """
    Example geometric cost:
        J = alpha * L(path) + beta / (clr_min(path) + eps)
    """
    L = metric.path_length(path)
    clr_min = path_clearance(path, dist_to_obstacles)
    eps = 1e-6
    return float(alpha * L + beta / (clr_min + eps))
      
