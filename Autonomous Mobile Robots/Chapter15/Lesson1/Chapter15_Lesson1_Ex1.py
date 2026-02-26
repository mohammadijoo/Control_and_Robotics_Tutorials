"""
Chapter15_Lesson1_Ex1.py
Exercise: Compute the lookahead point by intersecting a circle (radius Ld around robot)
with each path segment and selecting the "forward-most" intersection along the path.

This mirrors a common "circle intersection" implementation of pure pursuit.

Dependencies: numpy
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float


def circle_segment_intersections(c: np.ndarray, r: float, a: np.ndarray, b: np.ndarray) -> List[np.ndarray]:
    """
    Intersections of circle (center c, radius r) with segment [a,b].
    Return list of points (0,1,2 solutions) that lie on the segment.
    """
    d = b - a
    f = a - c
    A = float(np.dot(d, d))
    B = 2.0 * float(np.dot(f, d))
    C = float(np.dot(f, f) - r * r)

    disc = B * B - 4.0 * A * C
    if disc < 0.0 or A < 1e-12:
        return []
    disc_sqrt = math.sqrt(max(0.0, disc))

    t1 = (-B - disc_sqrt) / (2.0 * A)
    t2 = (-B + disc_sqrt) / (2.0 * A)

    pts = []
    for t in (t1, t2):
        if 0.0 <= t <= 1.0:
            pts.append(a + t * d)
    return pts


def lookahead_by_circle(path: np.ndarray, progress_idx: int, pose: Pose2D, Ld: float) -> Optional[Tuple[np.ndarray, int]]:
    """
    Starting from progress_idx, find the first segment intersection with the lookahead circle.
    Return (p_look, seg_index) or None if no intersection (e.g., near end).
    """
    c = np.array([pose.x, pose.y], dtype=float)
    for i in range(progress_idx, len(path) - 1):
        pts = circle_segment_intersections(c, Ld, path[i], path[i + 1])
        if pts:
            # choose the intersection closer to the end of the segment (more "forward")
            if len(pts) == 1:
                return pts[0], i
            # pick point with larger projection along segment direction
            d = path[i + 1] - path[i]
            scores = [float(np.dot(p - path[i], d)) for p in pts]
            return pts[int(np.argmax(scores))], i
    return None


def demo():
    # Simple path: straight then turn
    path = np.array([[0.0, 0.0],
                     [5.0, 0.0],
                     [8.0, 2.0],
                     [10.0, 5.0]], dtype=float)

    pose = Pose2D(x=1.0, y=-1.0, theta=0.0)
    Ld = 2.5

    out = lookahead_by_circle(path, progress_idx=0, pose=pose, Ld=Ld)
    print("Lookahead:", out)


if __name__ == "__main__":
    demo()
