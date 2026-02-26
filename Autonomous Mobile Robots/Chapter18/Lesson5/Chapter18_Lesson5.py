# Chapter18_Lesson5.py
# Case Study: Agricultural / Delivery Robots
# Risk-aware mission scoring with energy, time, and traversability terms

from dataclasses import dataclass
from typing import List, Tuple
import math
import heapq


@dataclass
class Edge:
    u: int
    v: int
    length_m: float
    slope_rad: float
    roughness: float        # 0..1
    slip_risk: float        # 0..1
    speed_limit: float      # m/s


def edge_time(edge: Edge, v_cmd: float) -> float:
    v = max(0.2, min(v_cmd, edge.speed_limit))
    return edge.length_m / v


def edge_energy(edge: Edge, mass: float, v_cmd: float, c_rr: float = 0.03,
                rho_air: float = 1.2, CdA: float = 0.45, P_aux: float = 80.0) -> float:
    v = max(0.2, min(v_cmd, edge.speed_limit))
    F_roll = c_rr * mass * 9.81 * math.cos(edge.slope_rad)
    F_grade = mass * 9.81 * math.sin(edge.slope_rad)
    F_drag = 0.5 * rho_air * CdA * v * v
    P = max(0.0, (F_roll + F_grade + F_drag) * v) + P_aux
    return P * (edge.length_m / v)  # Joules


def edge_risk(edge: Edge) -> float:
    # Simple normalized traversability risk score
    return 0.45 * edge.roughness + 0.55 * edge.slip_risk + 0.15 * abs(edge.slope_rad)


def stopping_distance(v: float, reaction_s: float, mu: float, margin: float = 0.5) -> float:
    return v * reaction_s + v * v / (2.0 * mu * 9.81) + margin


def safe_speed_from_clearance(clearance_m: float, reaction_s: float, mu: float, margin: float = 0.5) -> float:
    # Solve v*t_r + v^2/(2*mu*g) + margin <= clearance
    d = max(0.0, clearance_m - margin)
    a = 1.0 / (2.0 * mu * 9.81)
    b = reaction_s
    c = -d
    disc = max(0.0, b * b - 4 * a * c)
    return (-b + math.sqrt(disc)) / (2 * a)


def build_graph(edges: List[Edge], n_nodes: int):
    g = [[] for _ in range(n_nodes)]
    for e in edges:
        g[e.u].append(e)
    return g


def shortest_path(edges: List[Edge], n_nodes: int, source: int, target: int,
                  mass: float, v_cmd: float, w_t: float, w_e: float, w_r: float,
                  reaction_s: float, mu: float, clearance_default: float = 3.0):
    g = build_graph(edges, n_nodes)
    dist = [float("inf")] * n_nodes
    parent = [-1] * n_nodes
    dist[source] = 0.0
    pq = [(0.0, source)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        if u == target:
            break
        for e in g[u]:
            v_safe = safe_speed_from_clearance(clearance_default, reaction_s, mu)
            v_use = min(v_cmd, e.speed_limit, max(0.25, v_safe))
            c = (
                w_t * edge_time(e, v_use)
                + w_e * edge_energy(e, mass, v_use) / 1000.0  # kJ scaling
                + w_r * edge_risk(e)
            )
            nd = d + c
            if nd < dist[e.v]:
                dist[e.v] = nd
                parent[e.v] = u
                heapq.heappush(pq, (nd, e.v))

    # Reconstruct
    path = []
    cur = target
    if parent[cur] == -1 and cur != source:
        return [], float("inf")
    while cur != -1:
        path.append(cur)
        cur = parent[cur]
    path.reverse()
    return path, dist[target]


def agricultural_coverage_metrics(field_length: float, field_width: float,
                                  tool_width: float, overlap_ratio: float,
                                  turn_loss_s: float, cruise_speed: float) -> Tuple[int, float, float]:
    w_eff = tool_width * (1.0 - overlap_ratio)
    n_pass = math.ceil(field_width / w_eff)
    travel_dist = n_pass * field_length
    time_s = travel_dist / max(0.2, cruise_speed) + max(0, n_pass - 1) * turn_loss_s
    covered_area = field_length * field_width
    return n_pass, time_s, covered_area


def delivery_eta_chance_constraint(mean_times: List[float], std_times: List[float],
                                   deadline_s: float, z_quantile: float = 1.645) -> bool:
    mu = sum(mean_times)
    sigma = math.sqrt(sum(s * s for s in std_times))
    return (mu + z_quantile * sigma) <= deadline_s


if __name__ == "__main__":
    # Example graph shared by both robot types
    edges = [
        Edge(0, 1, 25,  0.02, 0.10, 0.15, 2.5),
        Edge(1, 2, 18,  0.05, 0.30, 0.35, 1.8),
        Edge(0, 3, 20, -0.01, 0.08, 0.10, 2.0),
        Edge(3, 2, 24,  0.03, 0.12, 0.20, 2.2),
        Edge(2, 4, 30,  0.01, 0.15, 0.18, 3.0),
        Edge(3, 4, 28,  0.07, 0.40, 0.45, 1.5),
    ]

    # Agricultural robot: heavier, risk and energy weighted
    ag_path, ag_cost = shortest_path(
        edges, n_nodes=5, source=0, target=4,
        mass=180.0, v_cmd=1.7, w_t=1.0, w_e=0.03, w_r=8.0,
        reaction_s=0.4, mu=0.55, clearance_default=2.5
    )
    print("Agricultural path:", ag_path, "cost:", round(ag_cost, 3))

    passes, t_cov, area = agricultural_coverage_metrics(
        field_length=120.0, field_width=48.0, tool_width=2.4,
        overlap_ratio=0.12, turn_loss_s=7.0, cruise_speed=1.4
    )
    print("Coverage passes:", passes, "time [min]:", round(t_cov / 60.0, 2), "area [m^2]:", area)

    # Delivery robot: lighter, time weighted, stricter deadline
    del_path, del_cost = shortest_path(
        edges, n_nodes=5, source=0, target=4,
        mass=45.0, v_cmd=2.2, w_t=1.5, w_e=0.01, w_r=5.0,
        reaction_s=0.6, mu=0.65, clearance_default=3.5
    )
    print("Delivery path:", del_path, "cost:", round(del_cost, 3))

    mean_segments = [110.0, 95.0, 150.0]
    std_segments = [12.0, 10.0, 20.0]
    ok = delivery_eta_chance_constraint(mean_segments, std_segments, deadline_s=390.0)
    print("Delivery ETA chance constraint satisfied:", ok)

    # Demonstrate safety speed bound
    vmax = safe_speed_from_clearance(clearance_m=4.0, reaction_s=0.5, mu=0.6, margin=0.6)
    print("Safe speed from clearance [m/s]:", round(vmax, 3))
