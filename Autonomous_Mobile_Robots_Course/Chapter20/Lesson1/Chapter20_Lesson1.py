# Chapter20_Lesson1.py
"""
Chapter 20 - Lesson 1
Problem Definition and Environment Setup for a Full AMR Autonomy Capstone

This script builds a reproducible capstone environment configuration:
1) mission specification
2) occupancy + traversability grid generation
3) sensor noise / update-rate definitions
4) feasibility checks (battery, timing, map bounds)
5) export of a JSON scenario file for later lessons
"""

from __future__ import annotations
from dataclasses import dataclass, asdict
from typing import Dict, Tuple
import json
import numpy as np


@dataclass
class MissionSpec:
    name: str
    start_xytheta: Tuple[float, float, float]
    goal_xy: Tuple[float, float]
    max_time_s: float
    min_success_prob: float
    max_collisions: int
    map_resolution_m: float


@dataclass
class SensorSuite:
    lidar_range_m: float
    lidar_fov_deg: float
    lidar_rate_hz: float
    imu_rate_hz: float
    wheel_rate_hz: float
    gps_rate_hz: float
    gps_available: bool


@dataclass
class RobotLimits:
    radius_m: float
    v_max_mps: float
    w_max_radps: float
    battery_wh: float
    avg_power_w: float


@dataclass
class EnvironmentConfig:
    width_m: float
    height_m: float
    obstacle_density: float
    terrain_classes: int
    slip_regions: int
    seed: int


def create_occupancy_grid(cfg: EnvironmentConfig, resolution_m: float) -> np.ndarray:
    nx = int(np.ceil(cfg.width_m / resolution_m))
    ny = int(np.ceil(cfg.height_m / resolution_m))
    rng = np.random.default_rng(cfg.seed)
    occ = (rng.random((ny, nx)) < cfg.obstacle_density).astype(np.uint8)

    # Clear a margin around the map border for safe spawn/goal placement
    margin = max(2, int(np.ceil(0.8 / resolution_m)))
    occ[:margin, :] = 0
    occ[-margin:, :] = 0
    occ[:, :margin] = 0
    occ[:, -margin:] = 0
    return occ


def create_traversability_map(occ: np.ndarray, cfg: EnvironmentConfig) -> np.ndarray:
    rng = np.random.default_rng(cfg.seed + 10)
    # 1.0 = easy terrain, larger values = harder terrain
    trav = 1.0 + 0.5 * rng.random(occ.shape)

    # Add a few slip-prone regions (higher cost)
    h, w = occ.shape
    for _ in range(cfg.slip_regions):
        cx = rng.integers(low=0, high=w)
        cy = rng.integers(low=0, high=h)
        rad = rng.integers(low=max(3, min(h, w) // 20), high=max(4, min(h, w) // 8))
        yy, xx = np.ogrid[:h, :w]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= rad ** 2
        trav[mask] += 1.0 + 0.8 * rng.random()

    # Obstacles are non-traversable
    trav[occ == 1] = np.inf
    return trav


def place_start_and_goal(occ: np.ndarray, resolution_m: float, seed: int) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    rng = np.random.default_rng(seed + 20)
    free = np.argwhere(occ == 0)
    if len(free) < 2:
        raise RuntimeError("Not enough free cells to place start and goal")

    # Try until distance is sufficiently long for a capstone run
    for _ in range(2000):
        i, j = rng.choice(len(free), size=2, replace=False)
        sy, sx = free[i]
        gy, gx = free[j]
        ds = np.hypot((gx - sx) * resolution_m, (gy - sy) * resolution_m)
        if ds >= 0.4 * max(occ.shape) * resolution_m:
            return (float(sx * resolution_m), float(sy * resolution_m)), (float(gx * resolution_m), float(gy * resolution_m))

    sy, sx = free[0]
    gy, gx = free[-1]
    return (float(sx * resolution_m), float(sy * resolution_m)), (float(gx * resolution_m), float(gy * resolution_m))


def battery_time_feasibility(robot: RobotLimits, mission: MissionSpec) -> Dict[str, float]:
    available_time_s = (robot.battery_wh / robot.avg_power_w) * 3600.0
    return {
        "available_time_s": available_time_s,
        "mission_time_s": mission.max_time_s,
        "time_margin_s": available_time_s - mission.max_time_s,
        "is_feasible": float(available_time_s >= mission.max_time_s),
    }


def sensor_schedule_check(sensors: SensorSuite) -> Dict[str, float]:
    # Basic consistency rules for later fusion and control
    checks = {
        "wheel_vs_control_ok": float(sensors.wheel_rate_hz >= 20.0),
        "imu_vs_fusion_ok": float(sensors.imu_rate_hz >= 50.0),
        "lidar_vs_localization_ok": float(sensors.lidar_rate_hz >= 5.0),
        "gps_flag": float(sensors.gps_available),
    }
    return checks


def export_scenario(path: str, mission: MissionSpec, sensors: SensorSuite, robot: RobotLimits,
                    env: EnvironmentConfig, occ: np.ndarray, trav: np.ndarray) -> None:
    payload = {
        "mission": asdict(mission),
        "sensors": asdict(sensors),
        "robot": asdict(robot),
        "environment": asdict(env),
        "occupancy_shape": list(occ.shape),
        "occupancy_occupied_fraction": float(np.mean(occ)),
        "traversability_finite_mean": float(np.mean(trav[np.isfinite(trav)])),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def main() -> None:
    env = EnvironmentConfig(
        width_m=60.0,
        height_m=40.0,
        obstacle_density=0.14,
        terrain_classes=3,
        slip_regions=4,
        seed=42,
    )

    robot = RobotLimits(
        radius_m=0.25,
        v_max_mps=1.2,
        w_max_radps=1.8,
        battery_wh=180.0,
        avg_power_w=85.0,
    )

    sensors = SensorSuite(
        lidar_range_m=15.0,
        lidar_fov_deg=270.0,
        lidar_rate_hz=10.0,
        imu_rate_hz=100.0,
        wheel_rate_hz=50.0,
        gps_rate_hz=5.0,
        gps_available=True,
    )

    # Initial dummy mission; start and goal are updated after map creation
    mission = MissionSpec(
        name="Capstone_AMR_IndoorOutdoor_Mix",
        start_xytheta=(0.0, 0.0, 0.0),
        goal_xy=(1.0, 1.0),
        max_time_s=1200.0,
        min_success_prob=0.90,
        max_collisions=0,
        map_resolution_m=0.2,
    )

    occ = create_occupancy_grid(env, mission.map_resolution_m)
    trav = create_traversability_map(occ, env)
    start_xy, goal_xy = place_start_and_goal(occ, mission.map_resolution_m, env.seed)

    mission = MissionSpec(
        name=mission.name,
        start_xytheta=(start_xy[0], start_xy[1], 0.0),
        goal_xy=goal_xy,
        max_time_s=mission.max_time_s,
        min_success_prob=mission.min_success_prob,
        max_collisions=mission.max_collisions,
        map_resolution_m=mission.map_resolution_m,
    )

    bt = battery_time_feasibility(robot, mission)
    sched = sensor_schedule_check(sensors)

    print("=== Capstone Mission Spec ===")
    print(mission)
    print("\n=== Battery / Time Feasibility ===")
    print(bt)
    print("\n=== Sensor Schedule Checks ===")
    print(sched)

    export_scenario("Chapter20_Lesson1_scenario.json", mission, sensors, robot, env, occ, trav)
    np.save("Chapter20_Lesson1_occupancy.npy", occ)
    np.save("Chapter20_Lesson1_traversability.npy", trav)
    print("\nFiles written: Chapter20_Lesson1_scenario.json, Chapter20_Lesson1_occupancy.npy, Chapter20_Lesson1_traversability.npy")


if __name__ == "__main__":
    main()
