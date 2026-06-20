import numpy as np
from enum import Enum
from dataclasses import dataclass

class Assumption(Enum):
    IDEAL = 0
    OFFSET = 1
    LINEARIZED = 2

@dataclass
class Planar2RModel:
    l1: float = 1.0
    l2: float = 1.0
    q_offset: np.ndarray = np.zeros(2)  # encoder offsets

    def fk(self, q, assumption=Assumption.IDEAL):
        """
        Simple 2R planar kinematics under different modeling assumptions.
        q: iterable of length 2, q = [q1, q2].
        Returns: np.array([x, y])
        """
        q = np.asarray(q, dtype=float).reshape(2,)
        if assumption == Assumption.OFFSET:
            q_eff = q + self.q_offset
        else:
            q_eff = q

        q1, q2 = q_eff

        if assumption == Assumption.LINEARIZED:
            # Linearize about q* = [0, 0]
            # Exact model: x = l1 cos(q1) + l2 cos(q1+q2)
            #             y = l1 sin(q1) + l2 sin(q1+q2)
            # First-order Taylor at q* gives:
            # x ≈ l1 + l2
            # y ≈ l1 q1 + l2 (q1 + q2)
            x = self.l1 + self.l2
            y = self.l1 * q1 + self.l2 * (q1 + q2)
        else:
            x = self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2)
            y = self.l1 * np.sin(q1) + self.l2 * np.sin(q1 + q2)

        return np.array([x, y])


if __name__ == "__main__":
    model = Planar2RModel(l1=1.0, l2=1.0, q_offset=np.array([0.05, -0.02]))
    q = np.deg2rad([10.0, 20.0])

    p_ideal = model.fk(q, Assumption.IDEAL)
    p_offset = model.fk(q, Assumption.OFFSET)
    p_lin = model.fk(q, Assumption.LINEARIZED)

    print("Ideal:", p_ideal)
    print("With offsets:", p_offset)
    print("Linearized:", p_lin)
    print("Offset error norm:", np.linalg.norm(p_offset - p_ideal))
    print("Linearization error norm:", np.linalg.norm(p_lin - p_ideal))
      
