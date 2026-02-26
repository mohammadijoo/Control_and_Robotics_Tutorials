# Chapter3_Lesson3.py
# Autonomous Mobile Robots — Chapter 3 Lesson 3
# Feasible Path Families for Car-Like Robots (Dubins + forward/backward variants)
#
# Requirements (Python): numpy, matplotlib
# Optional (library demo): ompl (python bindings) for exact Reeds–Shepp paths.

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np


def mod2pi(theta: float) -> float:
    return theta % (2.0 * math.pi)


def angdiff(a: float, b: float) -> float:
    """Minimal signed angle difference a-b in [-pi, pi)."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    theta: float  # radians


@dataclass(frozen=True)
class DubinsPath:
    path_type: str                  # one of: LSL, RSR, LSR, RSL, RLR, LRL
    segment_types: Tuple[str, str, str]  # each of: 'L','R','S'
    params: Tuple[float, float, float]   # (t, p, q) where turns are angles [rad], straight is length in normalized units
    R_min: float                    # minimum turning radius

    @property
    def length(self) -> float:
        t, p, q = self.params
        return self.R_min * (t + p + q)

    def sample(self, q0: Pose2, step: float = 0.1) -> np.ndarray:
        """Sample (x,y,theta) along the path, in world frame."""
        assert step > 0.0
        t, p, q = self.params
        segs = list(zip(self.segment_types, [t, p, q]))

        pts: List[Tuple[float, float, float]] = [(q0.x, q0.y, mod2pi(q0.theta))]
        x, y, th = q0.x, q0.y, mod2pi(q0.theta)

        for seg_type, seg_len in segs:
            if seg_type == 'S':
                L = seg_len * self.R_min
                n = max(1, int(math.ceil(L / step)))
                ds = L / n
                for _ in range(n):
                    x += ds * math.cos(th)
                    y += ds * math.sin(th)
                    pts.append((x, y, th))
            else:
                # turning: seg_len is angle, arc length = angle * R_min
                a = seg_len
                arc = a * self.R_min
                n = max(1, int(math.ceil(arc / step)))
                da = a / n
                for _ in range(n):
                    if seg_type == 'L':
                        x += self.R_min * (math.sin(th + da) - math.sin(th))
                        y += self.R_min * (-math.cos(th + da) + math.cos(th))
                        th = mod2pi(th + da)
                    else:  # 'R'
                        x += self.R_min * (-math.sin(th - da) + math.sin(th))
                        y += self.R_min * (math.cos(th - da) - math.cos(th))
                        th = mod2pi(th - da)
                    pts.append((x, y, th))

        return np.asarray(pts, dtype=float)


def _dubins_segment_unit(pose: Tuple[float, float, float], seg_type: str, seg_len: float) -> Tuple[float, float, float]:
    """Exact integration for unit-radius primitives in the canonical (normalized) frame."""
    x, y, th = pose
    if seg_type == 'S':
        x += seg_len * math.cos(th)
        y += seg_len * math.sin(th)
        return (x, y, th)
    if seg_type == 'L':
        x += math.sin(th + seg_len) - math.sin(th)
        y += -math.cos(th + seg_len) + math.cos(th)
        th = mod2pi(th + seg_len)
        return (x, y, th)
    if seg_type == 'R':
        x += -math.sin(th - seg_len) + math.sin(th)
        y += math.cos(th - seg_len) - math.cos(th)
        th = mod2pi(th - seg_len)
        return (x, y, th)
    raise ValueError(f"Unknown seg_type: {seg_type}")


def _endpoint_error(alpha: float, d: float, beta: float, seg_types: Tuple[str, str, str], params: Tuple[float, float, float]) -> float:
    pose = (0.0, 0.0, alpha)  # start in canonical frame
    for st, sl in zip(seg_types, params):
        pose = _dubins_segment_unit(pose, st, sl)
    x, y, th = pose
    return math.hypot(x - d, y) + abs(angdiff(th, beta))


def _LSL(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    tmp0 = d + sa - sb
    p2 = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sa - sb)
    if p2 < 0.0:
        return None
    p = math.sqrt(p2)
    tmp1 = math.atan2((cb - ca), tmp0)
    t = mod2pi(-alpha + tmp1)
    q = mod2pi(beta - tmp1)
    return (t, p, q)


def _RSR(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    tmp0 = d - sa + sb
    p2 = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (-sa + sb)
    if p2 < 0.0:
        return None
    p = math.sqrt(p2)
    tmp1 = math.atan2((ca - cb), tmp0)
    t = mod2pi(alpha - tmp1)
    q = mod2pi(-beta + tmp1)
    return (t, p, q)


def _LSR(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p2 = -2.0 + d * d + 2.0 * c_ab + 2.0 * d * (sa + sb)
    if p2 < 0.0:
        return None
    p = math.sqrt(p2)
    tmp0 = math.atan2((-ca - cb), (d + sa + sb))
    tmp1 = math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp0 - tmp1)
    q = mod2pi(-beta + tmp0 - tmp1)
    return (t, p, q)


def _RSL(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p2 = -2.0 + d * d + 2.0 * c_ab - 2.0 * d * (sa + sb)
    if p2 < 0.0:
        return None
    p = math.sqrt(p2)
    tmp0 = math.atan2((ca + cb), (d - sa - sb))
    tmp1 = math.atan2(2.0, p)
    t = mod2pi(alpha - tmp0 + tmp1)
    q = mod2pi(beta - tmp0 + tmp1)
    return (t, p, q)


def _RLR(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    tmp0 = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp0) > 1.0:
        return None
    p = mod2pi(2.0 * math.pi - math.acos(tmp0))
    tmp1 = math.atan2((ca - cb), (d - sa + sb))
    t = mod2pi(alpha - tmp1 + p / 2.0)
    q = mod2pi(alpha - beta - t + p)
    return (t, p, q)


def _LRL(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    # symmetry (mirror across x-axis): LRL(alpha,beta,d) = RLR(-alpha,-beta,d)
    res = _RLR(mod2pi(-alpha), mod2pi(-beta), d)
    if res is None:
        return None
    return res


_DUBINS_CANDIDATES = {
    "LSL": (('L', 'S', 'L'), _LSL),
    "RSR": (('R', 'S', 'R'), _RSR),
    "LSR": (('L', 'S', 'R'), _LSR),
    "RSL": (('R', 'S', 'L'), _RSL),
    "RLR": (('R', 'L', 'R'), _RLR),
    "LRL": (('L', 'R', 'L'), _LRL),
}


def dubins_shortest_path(q0: Pose2, q1: Pose2, R_min: float) -> DubinsPath:
    """Compute the shortest Dubins path (forward-only, bounded curvature)."""
    if R_min <= 0.0:
        raise ValueError("R_min must be positive")

    # 1) translate/rotate so that q0 is at origin and heading is 0
    dx = q1.x - q0.x
    dy = q1.y - q0.y
    c0, s0 = math.cos(q0.theta), math.sin(q0.theta)
    x = (c0 * dx + s0 * dy) / R_min
    y = (-s0 * dx + c0 * dy) / R_min
    phi = mod2pi(q1.theta - q0.theta)

    # 2) rotate by theta so goal is on +x axis at (d,0)
    d = math.hypot(x, y)
    theta = math.atan2(y, x) if d > 0.0 else 0.0
    alpha = mod2pi(-theta)
    beta = mod2pi(phi - theta)

    best: Optional[DubinsPath] = None
    best_len = float('inf')

    for name, (seg_types, fn) in _DUBINS_CANDIDATES.items():
        params = fn(alpha, beta, d)
        if params is None:
            continue
        err = _endpoint_error(alpha, d, beta, seg_types, params)
        if err > 1e-6:
            continue
        L = R_min * (params[0] + params[1] + params[2])
        if L < best_len:
            best_len = L
            best = DubinsPath(name, seg_types, params, R_min)

    if best is None:
        raise RuntimeError("No feasible Dubins path found; check inputs")

    return best


def dubins_backward_only_path(q0: Pose2, q1: Pose2, R_min: float) -> DubinsPath:
    """A simple reverse-driving variant: drive backward along a Dubins path.

    Driving backward with curvature bound is equivalent to driving forward from
    (x,y,theta+pi) to (x',y',theta'+pi).
    This does NOT allow direction switches (unlike full Reeds–Shepp).
    """
    q0b = Pose2(q0.x, q0.y, mod2pi(q0.theta + math.pi))
    q1b = Pose2(q1.x, q1.y, mod2pi(q1.theta + math.pi))
    return dubins_shortest_path(q0b, q1b, R_min)


def demo_plot() -> None:
    import matplotlib.pyplot as plt

    q0 = Pose2(0.0, 0.0, math.radians(10))
    q1 = Pose2(8.0, 4.0, math.radians(110))
    R_min = 2.0

    path_fwd = dubins_shortest_path(q0, q1, R_min)
    pts_fwd = path_fwd.sample(q0, step=0.05)

    path_back = dubins_backward_only_path(q0, q1, R_min)
    pts_back = path_back.sample(Pose2(q0.x, q0.y, mod2pi(q0.theta + math.pi)), step=0.05)

    print("Forward-only Dubins:", path_fwd.path_type, "length=", path_fwd.length)
    print("Backward-only Dubins (all reverse):", path_back.path_type, "length=", path_back.length)

    plt.figure()
    plt.plot(pts_fwd[:, 0], pts_fwd[:, 1], label=f"Dubins forward ({path_fwd.path_type})")
    plt.plot(pts_back[:, 0], pts_back[:, 1], label=f"Dubins backward-only ({path_back.path_type})")
    plt.scatter([q0.x, q1.x], [q0.y, q1.y])
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("Curvature-bounded feasible paths (Dubins)")
    plt.show()


def demo_reeds_shepp_with_ompl() -> None:
    """Optional: exact Reeds–Shepp path via OMPL (if installed).

    pip install ompl (availability depends on platform) or build OMPL bindings.
    """
    try:
        from ompl import base as ob
        from ompl import geometric as og
    except Exception as e:
        print("OMPL is not available in this environment:", e)
        return

    q0 = Pose2(0.0, 0.0, math.radians(10))
    q1 = Pose2(8.0, 4.0, math.radians(110))
    R_min = 2.0

    space = ob.ReedsSheppStateSpace(R_min)
    si = ob.SpaceInformation(space)

    start = ob.State(space)
    goal = ob.State(space)
    start().setXY(q0.x, q0.y)
    start().setYaw(q0.theta)
    goal().setXY(q1.x, q1.y)
    goal().setYaw(q1.theta)

    rs = space.reedsShepp(start(), goal())
    print("Reeds–Shepp length (OMPL):", rs.length())

    # Sample the path:
    states = []
    for s in np.linspace(0.0, rs.length(), 200):
        st = ob.State(space)
        space.interpolate(start(), goal(), s / rs.length(), st())
        states.append((st().getX(), st().getY(), st().getYaw()))
    states = np.asarray(states)

    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(states[:, 0], states[:, 1])
    plt.axis('equal')
    plt.grid(True)
    plt.title("Reeds–Shepp path (OMPL sampling)")
    plt.show()


if __name__ == "__main__":
    demo_plot()
    # demo_reeds_shepp_with_ompl()
