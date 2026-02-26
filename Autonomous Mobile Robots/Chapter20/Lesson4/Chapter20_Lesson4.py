"""
Chapter20_Lesson4.py
Navigation Stack Deployment (Capstone AMR)
Educational deployment supervisor + timing monitor + local scoring example.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Tuple
import math
import time
import random


class NodeState(Enum):
    UNCONFIGURED = auto()
    INACTIVE = auto()
    ACTIVE = auto()
    ERROR = auto()


@dataclass
class ManagedNode:
    name: str
    startup_ms: float
    state: NodeState = NodeState.UNCONFIGURED

    def configure(self) -> bool:
        time.sleep(self.startup_ms / 1000.0)
        self.state = NodeState.INACTIVE
        return True

    def activate(self) -> bool:
        if self.state != NodeState.INACTIVE:
            self.state = NodeState.ERROR
            return False
        self.state = NodeState.ACTIVE
        return True

    def deactivate(self) -> bool:
        if self.state == NodeState.ACTIVE:
            self.state = NodeState.INACTIVE
        return True

    def reset(self) -> bool:
        self.state = NodeState.UNCONFIGURED
        return True


@dataclass
class TimingBudget:
    period_ms: float
    wcet_ms: Dict[str, float]

    def utilization(self) -> float:
        return sum(v / self.period_ms for v in self.wcet_ms.values())

    def is_schedulable(self) -> bool:
        # Simplified single-core utilization check
        return self.utilization() <= 1.0

    def report(self) -> str:
        total = sum(self.wcet_ms.values())
        return (
            f"Period={self.period_ms:.1f} ms | Total WCET={total:.1f} ms | "
            f"Utilization={self.utilization():.3f} | "
            f"Schedulable={self.is_schedulable()}"
        )


@dataclass
class CostmapParams:
    inscribed_radius: float = 0.22
    inflation_radius: float = 0.70
    cost_lethal: int = 254
    cost_inscribed: int = 220
    kappa: float = 8.0  # exponential decay rate

    def inflated_cost(self, d: float) -> int:
        """Distance d from obstacle center to robot center."""
        if d <= self.inscribed_radius:
            return self.cost_lethal
        if d >= self.inflation_radius:
            return 0
        val = self.cost_inscribed * math.exp(-self.kappa * (d - self.inscribed_radius))
        return int(max(1, min(self.cost_inscribed, round(val))))


@dataclass
class LocalPlannerWeights:
    w_path: float = 1.4
    w_goal: float = 1.0
    w_obstacle: float = 2.6
    w_velocity: float = 0.4
    w_spin: float = 0.2


def score_trajectory(
    dist_to_path: float,
    dist_to_goal: float,
    max_obstacle_cost: int,
    v: float,
    omega: float,
    v_ref: float,
    weights: LocalPlannerWeights
) -> float:
    """Lower score is better."""
    obstacle_term = max_obstacle_cost / 254.0
    return (
        weights.w_path * dist_to_path
        + weights.w_goal * dist_to_goal
        + weights.w_obstacle * obstacle_term
        + weights.w_velocity * abs(v - v_ref)
        + weights.w_spin * abs(omega)
    )


@dataclass
class DeploymentSupervisor:
    nodes: List[ManagedNode]
    timing: TimingBudget
    localization_cov_threshold: float = 0.20 ** 2   # m^2
    max_consecutive_planner_misses: int = 3
    planner_deadline_ms: float = 80.0

    planner_miss_count: int = 0
    active: bool = False
    mission_log: List[str] = field(default_factory=list)

    def startup(self) -> bool:
        self.mission_log.append("Startup sequence begin")
        if not self.timing.is_schedulable():
            self.mission_log.append("Timing budget failed")
            return False

        for node in self.nodes:
            if not node.configure():
                self.mission_log.append(f"Configure failed: {node.name}")
                return False

        for node in self.nodes:
            if not node.activate():
                self.mission_log.append(f"Activate failed: {node.name}")
                return False

        self.active = True
        self.mission_log.append("Startup sequence complete")
        return True

    def check_localization_health(self, cov_xx: float, cov_yy: float) -> bool:
        trace_xy = cov_xx + cov_yy
        ok = trace_xy <= self.localization_cov_threshold
        self.mission_log.append(
            f"Localization trace={trace_xy:.4f} m^2 -> {'OK' if ok else 'RECOVERY'}"
        )
        return ok

    def monitor_planner_deadline(self, planner_runtime_ms: float) -> bool:
        if planner_runtime_ms > self.planner_deadline_ms:
            self.planner_miss_count += 1
            self.mission_log.append(
                f"Planner deadline miss ({planner_runtime_ms:.1f} ms), count={self.planner_miss_count}"
            )
        else:
            self.planner_miss_count = 0
            self.mission_log.append(
                f"Planner runtime OK ({planner_runtime_ms:.1f} ms)"
            )

        if self.planner_miss_count >= self.max_consecutive_planner_misses:
            self.mission_log.append("Trigger recovery: clear local costmap + stop")
            self.planner_miss_count = 0
            return False
        return True

    def graceful_shutdown(self) -> None:
        self.mission_log.append("Shutdown begin")
        for node in reversed(self.nodes):
            node.deactivate()
            node.reset()
        self.active = False
        self.mission_log.append("Shutdown complete")


def simulate_deployment():
    nodes = [
        ManagedNode("map_server", 20),
        ManagedNode("amcl_or_ekf", 25),
        ManagedNode("global_planner", 15),
        ManagedNode("local_planner", 12),
        ManagedNode("controller_server", 10),
        ManagedNode("bt_navigator", 18),
    ]

    timing = TimingBudget(
        period_ms=100.0,
        wcet_ms={
            "sensor_preprocess": 8.0,
            "localization": 14.0,
            "costmap_update": 18.0,
            "global_planner": 12.0,
            "local_planner": 22.0,
            "controller": 5.0,
            "bt_tick": 3.0,
        },
    )

    sup = DeploymentSupervisor(nodes=nodes, timing=timing)
    print(timing.report())

    if not sup.startup():
        print("Deployment startup failed")
        print("\n".join(sup.mission_log))
        return

    # Example costmap inflation profile
    infl = CostmapParams()
    distances = [0.18, 0.25, 0.35, 0.50, 0.80]
    print("Inflation costs:", {d: infl.inflated_cost(d) for d in distances})

    # Example trajectory scoring
    w = LocalPlannerWeights()
    candidates = [
        (0.05, 1.20, 140, 0.40, 0.10),
        (0.18, 0.95,  80, 0.55, 0.35),
        (0.10, 1.00, 220, 0.50, 0.05),
    ]
    scored: List[Tuple[int, float]] = []
    for idx, c in enumerate(candidates):
        s = score_trajectory(*c, v_ref=0.50, weights=w)
        scored.append((idx, s))
    print("Candidate scores:", scored, "best=", min(scored, key=lambda x: x[1]))

    # Simulated runtime monitoring
    random.seed(4)
    for step in range(12):
        # Fake localization covariance (grows under degraded sensing)
        cov_xx = 0.006 + 0.002 * math.sin(step)
        cov_yy = 0.005 + 0.002 * math.cos(step)
        if step in (8, 9):
            cov_xx += 0.03
            cov_yy += 0.03
        if not sup.check_localization_health(cov_xx, cov_yy):
            sup.mission_log.append("Recovery action: relocalize / slow down")

        runtime_ms = 55 + 40 * random.random()
        sup.monitor_planner_deadline(runtime_ms)

    sup.graceful_shutdown()
    print("\n--- Mission Log ---")
    print("\n".join(sup.mission_log))


if __name__ == "__main__":
    simulate_deployment()
