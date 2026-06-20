import math
from dataclasses import dataclass

@dataclass
class SafetyEnvelope1D:
    reaction_time: float   # T_r
    max_decel: float       # a_max > 0
    no_go_distance: float  # d_ng safety radius around human
    margin: float = 0.05   # extra margin [m]

    def stopping_distance(self, v: float) -> float:
        """
        Compute d_stop = v*T_r + v^2/(2*a_max).
        Assumes v >= 0 and a_max > 0.
        """
        return v * self.reaction_time + v * v / (2.0 * self.max_decel)

    def is_safe(self, distance: float, v: float) -> bool:
        """
        Return True if current state (distance, v) is within the safety envelope.
        distance: current distance from human [m]
        v: speed toward the human [m/s]
        """
        if v < 0.0:
            # Moving away from the human is always safe w.r.t this constraint.
            return True
        d_available = max(distance - self.no_go_distance - self.margin, 0.0)
        d_required = self.stopping_distance(v)
        return d_required <= d_available

# Example usage:
if __name__ == "__main__":
    env = SafetyEnvelope1D(reaction_time=0.2, max_decel=3.0, no_go_distance=0.4)
    distance = 1.0  # meters
    v = 0.8         # m/s
    if env.is_safe(distance, v):
        print("Command accepted: within safety envelope.")
    else:
        print("Safety stop: requested speed is unsafe.")
      
