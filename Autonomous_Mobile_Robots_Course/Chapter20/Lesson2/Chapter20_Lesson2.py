# Chapter20_Lesson2.py
# Sensor Suite Selection (given hardware) - Capstone AMR
# Python reference implementation: constrained subset search using an information-form proxy.

import itertools
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

np.set_printoptions(precision=4, suppress=True)

@dataclass(frozen=True)
class SensorSpec:
    name: str
    rate_hz: float
    power_w: float
    bandwidth_kbps: float
    cpu_ms: float                 # processing time per measurement
    sigma: Tuple[float, ...]      # std dev per measurement channel
    H: np.ndarray                 # linearized measurement Jacobian
    mandatory: bool = False

def info_update(P: np.ndarray, H: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Information-form covariance update for linear measurement y = Hx + v."""
    I_prior = np.linalg.inv(P)
    I_meas = H.T @ np.linalg.inv(R) @ H
    return np.linalg.inv(I_prior + I_meas)

def suite_metrics(suite: List[SensorSpec], P0: np.ndarray, dt: float = 0.1) -> Dict[str, float]:
    # Process model proxy (drift per cycle) for state [x, y, theta, v, gyro_bias]
    Q = np.diag([0.03, 0.03, 0.01, 0.05, 0.002]) * dt
    P_pred = P0 + Q

    P = P_pred.copy()
    total_power = 0.0
    total_bw = 0.0
    cpu_load_ratio = 0.0
    latency_ms = 0.0

    for s in suite:
        H = s.H
        R = np.diag(np.array(s.sigma) ** 2)
        P = info_update(P, H, R)
        total_power += s.power_w
        total_bw += s.bandwidth_kbps
        cpu_load_ratio += (s.cpu_ms * s.rate_hz) / 1000.0
        latency_ms += (1000.0 / s.rate_hz) + s.cpu_ms  # sample period + processing

    # Scalar proxies
    trP = float(np.trace(P))
    logdet_info = float(np.log(np.linalg.det(np.linalg.inv(P))))
    mean_latency_ms = latency_ms / max(1, len(suite))
    return {
        "traceP": trP,
        "logdetInfo": logdet_info,
        "powerW": total_power,
        "bandwidthKbps": total_bw,
        "cpuLoad": cpu_load_ratio,
        "meanLatencyMs": mean_latency_ms,
    }

def score(metrics: Dict[str, float]) -> float:
    # Multi-objective weighted score (maximize)
    # Higher logdet information is better; lower resource use and latency are better.
    return (
        1.0 * metrics["logdetInfo"]
        - 0.35 * metrics["traceP"]
        - 0.03 * metrics["powerW"]
        - 0.0008 * metrics["bandwidthKbps"]
        - 1.8 * metrics["cpuLoad"]
        - 0.01 * metrics["meanLatencyMs"]
    )

def build_sensors() -> List[SensorSpec]:
    # State: x = [x, y, theta, v, b_g]^T
    # Measurement models are linearized proxies for selection analysis.
    sensors = [
        SensorSpec(
            name="wheel_encoder",
            rate_hz=50.0,
            power_w=0.5,
            bandwidth_kbps=20.0,
            cpu_ms=0.3,
            sigma=(0.03, 0.01),  # v, yaw-rate-like proxy
            H=np.array([
                [0, 0, 0, 1, 0],   # velocity
                [0, 0, 1, 0, 1],   # theta + gyro bias proxy
            ], dtype=float),
            mandatory=True
        ),
        SensorSpec(
            name="imu",
            rate_hz=200.0,
            power_w=0.8,
            bandwidth_kbps=120.0,
            cpu_ms=0.4,
            sigma=(0.015, 0.01),  # yaw-rate, accel/velocity proxy
            H=np.array([
                [0, 0, 1, 0, 1],   # yaw + bias
                [0, 0, 0, 1, 0],   # velocity
            ], dtype=float),
            mandatory=True
        ),
        SensorSpec(
            name="2d_lidar",
            rate_hz=10.0,
            power_w=8.0,
            bandwidth_kbps=1500.0,
            cpu_ms=12.0,
            sigma=(0.05, 0.05, 0.02),  # x, y, theta corrections from scan matching
            H=np.array([
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
            ], dtype=float),
        ),
        SensorSpec(
            name="mono_camera",
            rate_hz=30.0,
            power_w=2.5,
            bandwidth_kbps=2500.0,
            cpu_ms=18.0,
            sigma=(0.08, 0.08, 0.03),  # VO pose increment proxy
            H=np.array([
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
            ], dtype=float),
        ),
        SensorSpec(
            name="depth_camera",
            rate_hz=15.0,
            power_w=4.5,
            bandwidth_kbps=5000.0,
            cpu_ms=20.0,
            sigma=(0.06, 0.06, 0.03, 0.08),  # x, y, theta, v
            H=np.array([
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
            ], dtype=float),
        ),
        SensorSpec(
            name="gnss_rtk",
            rate_hz=5.0,
            power_w=2.0,
            bandwidth_kbps=40.0,
            cpu_ms=1.5,
            sigma=(0.03, 0.03),  # x, y absolute
            H=np.array([
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
            ], dtype=float),
        ),
    ]
    return sensors

def select_suite():
    sensors = build_sensors()
    P0 = np.diag([1.5, 1.5, 0.5, 0.8, 0.2])  # prior covariance at startup

    power_budget = 14.0
    bw_budget = 7000.0
    cpu_budget = 0.80   # 80% of one CPU core equivalent
    latency_budget_ms = 120.0

    mandatory = [s for s in sensors if s.mandatory]
    optional = [s for s in sensors if not s.mandatory]

    best = None
    evaluated = []

    for r in range(len(optional) + 1):
        for combo in itertools.combinations(optional, r):
            suite = mandatory + list(combo)
            metrics = suite_metrics(suite, P0)
            feasible = (
                metrics["powerW"] <= power_budget and
                metrics["bandwidthKbps"] <= bw_budget and
                metrics["cpuLoad"] <= cpu_budget and
                metrics["meanLatencyMs"] <= latency_budget_ms
            )
            if not feasible:
                continue
            J = score(metrics)
            item = (J, suite, metrics)
            evaluated.append(item)
            if best is None or J > best[0]:
                best = item

    if best is None:
        raise RuntimeError("No feasible sensor suite under the provided budgets.")

    # Sort top feasible suites
    evaluated.sort(key=lambda x: x[0], reverse=True)
    print("Top feasible suites:")
    for rank, (J, suite, metrics) in enumerate(evaluated[:8], start=1):
        names = [s.name for s in suite]
        print(f"{rank:2d}. score={J:.3f}  suite={names}")
        print("    ", {k: round(v, 4) for k, v in metrics.items()})

    print("\nBest suite recommendation:")
    J, suite, metrics = best
    print("Sensors:", [s.name for s in suite])
    print("Score:", round(J, 4))
    print("Metrics:", {k: round(v, 4) for k, v in metrics.items()})

if __name__ == "__main__":
    select_suite()
