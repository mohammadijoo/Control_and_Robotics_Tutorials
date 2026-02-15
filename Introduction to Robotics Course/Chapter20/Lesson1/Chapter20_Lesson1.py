from dataclasses import dataclass
from typing import Dict

@dataclass
class PerformanceRequirements:
    max_settling_time_s: float
    max_overshoot: float
    max_rms_error: float

@dataclass
class PerformanceMetrics:
    settling_time_s: float
    overshoot: float
    rms_error: float

def check_feasibility(req: PerformanceRequirements,
                      met: PerformanceMetrics) -> Dict[str, bool]:
    """
    Return a dictionary telling which constraints are satisfied.
    """
    return {
        "settling_time_ok": met.settling_time_s <= req.max_settling_time_s,
        "overshoot_ok": met.overshoot <= req.max_overshoot,
        "rms_error_ok": met.rms_error <= req.max_rms_error,
    }

if __name__ == "__main__":
    # Example numbers from a simple simulation of a line-following robot
    requirements = PerformanceRequirements(
        max_settling_time_s=2.0,
        max_overshoot=0.05,
        max_rms_error=0.02
    )

    metrics = PerformanceMetrics(
        settling_time_s=1.8,
        overshoot=0.04,
        rms_error=0.018
    )

    feasibility = check_feasibility(requirements, metrics)
    print("Feasibility report:")
    for key, ok in feasibility.items():
        print(f"  {key}: {'OK' if ok else 'VIOLATED'}")
      
