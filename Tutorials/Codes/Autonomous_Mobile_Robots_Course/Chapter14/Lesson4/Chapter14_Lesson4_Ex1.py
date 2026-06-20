"""
Chapter14_Lesson4_Ex1.py
Exercise: pure-Python progress + oscillation detector (no ROS).

Given:
- distance-to-goal samples d(t)
- cmd samples (v_x(t), w_z(t))

Compute:
- progress statistic over a sliding window
- oscillation score via sign flips
"""

from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class DetParams:
    window_s: float = 10.0
    min_progress_m: float = 0.25
    osc_window_s: float = 6.0
    osc_sign_flips: int = 6
    v_eps: float = 0.03

def _trim(hist: List[Tuple[float, ...]], window_s: float) -> List[Tuple[float, ...]]:
    if not hist:
        return hist
    t_latest = hist[-1][0]
    t_min = t_latest - window_s
    i = 0
    while i < len(hist) and hist[i][0] < t_min:
        i += 1
    return hist[i:]

def progress_ok(dist_hist: List[Tuple[float, float]], p: DetParams) -> bool:
    dist_hist = _trim(dist_hist, p.window_s)
    if len(dist_hist) < 2:
        return True
    d0 = dist_hist[0][1]
    d1 = dist_hist[-1][1]
    return (d0 - d1) >= p.min_progress_m

def oscillating(cmd_hist: List[Tuple[float, float, float]], p: DetParams) -> bool:
    cmd_hist = _trim(cmd_hist, p.osc_window_s)
    if len(cmd_hist) < 3:
        return False

    def sgn(v: float) -> int:
        if abs(v) < p.v_eps:
            return 0
        return 1 if v > 0 else -1

    sx = [sgn(vx) for _, vx, _ in cmd_hist]
    sz = [sgn(wz) for _, _, wz in cmd_hist]

    flips_x = sum(1 for i in range(1, len(sx)) if sx[i] != 0 and sx[i-1] != 0 and sx[i] != sx[i-1])
    flips_z = sum(1 for i in range(1, len(sz)) if sz[i] != 0 and sz[i-1] != 0 and sz[i] != sz[i-1])
    return (flips_x + flips_z) >= p.osc_sign_flips

if __name__ == "__main__":
    p = DetParams()

    # Toy example: d(t) stagnates, commands flip
    dist = [(0.0, 5.0), (5.0, 4.95), (10.0, 4.93)]
    cmd = [
        (0.0,  0.10,  0.0),
        (1.0, -0.10,  0.0),
        (2.0,  0.10,  0.0),
        (3.0, -0.10,  0.0),
        (4.0,  0.10,  0.0),
        (5.0, -0.10,  0.0),
    ]

    print("progress_ok:", progress_ok(dist, p))
    print("oscillating:", oscillating(cmd, p))
