import numpy as np
from dataclasses import dataclass
from typing import Protocol, Dict, Any

class TrajectoryRequirement(Protocol):
    def check(self, t: np.ndarray, signals: Dict[str, np.ndarray]) -> bool:
        ...

@dataclass
class MaxTrackingError(TrajectoryRequirement):
    ref_key: str
    out_key: str
    e_max: float

    def check(self, t: np.ndarray, signals: Dict[str, np.ndarray]) -> bool:
        r = signals[self.ref_key]
        y = signals[self.out_key]
        e = r - y
        e_norm = np.linalg.norm(e, axis=1)  # assume shape (N, p)
        return float(np.max(e_norm)) <= self.e_max

@dataclass
class MinSeparation(TrajectoryRequirement):
    r_key: str     # robot position
    h_key: str     # human position
    d_min: float

    def check(self, t: np.ndarray, signals: Dict[str, np.ndarray]) -> bool:
        p_r = signals[self.r_key]   # shape (N, 3)
        p_h = signals[self.h_key]   # shape (N, 3)
        d = np.linalg.norm(p_r - p_h, axis=1)
        return bool(np.all(d >= self.d_min))

def check_all_requirements(
    t: np.ndarray,
    signals: Dict[str, np.ndarray],
    requirements: Dict[str, TrajectoryRequirement]
) -> Dict[str, bool]:
    results = {}
    for name, req in requirements.items():
        results[name] = req.check(t, signals)
    return results

# Example usage with sampled trajectories:
N = 1000
t = np.linspace(0.0, 2.0, N)
# Fake 1D position reference and output
r = np.ones((N, 1))  # constant 1 m
y = r + 0.002 * np.sin(10.0 * t)  # small oscillation

# Fake positions for robot and human
p_r = np.column_stack([0.5 * np.ones(N), 0.0 * t, 0.0 * t])
p_h = np.column_stack([0.0 * t, 0.0 * t, 0.0 * t])

signals = {
    "ref": r,
    "y": y,
    "p_r": p_r,
    "p_h": p_h,
}

reqs = {
    "tracking": MaxTrackingError(ref_key="ref", out_key="y", e_max=0.01),
    "separation": MinSeparation(r_key="p_r", h_key="p_h", d_min=0.2),
}

results = check_all_requirements(t, signals, reqs)
for name, ok in results.items():
    print(f"{name}: {'OK' if ok else 'VIOLATED'}")
      
