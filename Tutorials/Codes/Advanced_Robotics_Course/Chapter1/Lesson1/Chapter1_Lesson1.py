import numpy as np

class CSpace:
    """
    Simple box-bounded configuration space with optional periodic coordinates.
    periodic is a boolean mask of same length as bounds.
    """
    def __init__(self, lower_bounds, upper_bounds, periodic=None):
        self.lower = np.array(lower_bounds, dtype=float)
        self.upper = np.array(upper_bounds, dtype=float)
        assert self.lower.shape == self.upper.shape
        self.n = self.lower.size
        if periodic is None:
            periodic = np.zeros(self.n, dtype=bool)
        self.periodic = np.array(periodic, dtype=bool)

    def _wrap(self, q):
        """Wrap periodic coordinates to (-pi, pi]."""
        q = np.array(q, dtype=float)
        for i in range(self.n):
            if self.periodic[i]:
                # Wrap to (-pi, pi]
                q[i] = (q[i] + np.pi) % (2.0 * np.pi) - np.pi
        return q

    def shortest_difference(self, q_from, q_to):
        """Compute shortest coordinate-wise difference on mixed periodic space."""
        q_from = self._wrap(q_from)
        q_to = self._wrap(q_to)
        diff = q_to - q_from
        for i in range(self.n):
            if self.periodic[i]:
                # Choose representation with smallest magnitude
                if diff[i] > np.pi:
                    diff[i] -= 2.0 * np.pi
                elif diff[i] < -np.pi:
                    diff[i] += 2.0 * np.pi
        return diff

    def interpolate(self, q_start, q_goal, num_samples):
        """
        Generate a discrete path from q_start to q_goal using linear interpolation
        in configuration space (respecting periodic coordinates).
        """
        q_start = self._wrap(q_start)
        diff = self.shortest_difference(q_start, q_goal)
        alphas = np.linspace(0.0, 1.0, num_samples)
        path = []
        for a in alphas:
            q = q_start + a * diff
            q = self._wrap(q)
            path.append(q)
        return np.vstack(path)

# Example usage: 2R planar arm with both joints periodic
cspace = CSpace(lower_bounds=[-np.pi, -np.pi],
                upper_bounds=[ np.pi,  np.pi],
                periodic=[True, True])

q_start = [0.0, 0.0]
q_goal  = [np.pi, -np.pi]  # same as [-pi, -pi] physically
path = cspace.interpolate(q_start, q_goal, num_samples=5)
print(path)
      
