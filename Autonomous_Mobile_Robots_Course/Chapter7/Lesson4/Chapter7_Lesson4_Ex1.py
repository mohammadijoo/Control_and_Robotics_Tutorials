"""
Chapter7_Lesson4_Ex1.py
Exercise: Joint scaling of Q and R using NIS and NEES targets.

Idea:
  - Use mean NIS to tune R scale (measurement confidence).
  - Use mean NEES to tune Q scale (process/model confidence).
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

# Reuse the EKF components by importing from Chapter7_Lesson4.py
# (Keep both files in the same folder.)
import Chapter7_Lesson4 as base


@dataclass
class Targets:
    nis_dof: int = 2
    nees_dof: int = 3


def tune_scales(p: base.NoiseParams, nis_list: List[float], nees_list: List[float], t: Targets) -> base.NoiseParams:
    nis_mean = float(np.mean(nis_list))
    nees_mean = float(np.mean(nees_list))

    # R scale update from NIS
    r_gain = nis_mean / float(t.nis_dof)
    r_gain = max(0.2, min(5.0, r_gain))

    # Q scale update from NEES:
    # If NEES too high (filter overconfident about model), increase Q.
    q_gain = nees_mean / float(t.nees_dof)
    q_gain = max(0.2, min(5.0, q_gain))

    p2 = base.NoiseParams(**{**p.__dict__})
    p2.r_scale *= r_gain
    p2.q_scale *= q_gain
    return p2


def main() -> None:
    dt = 0.1
    x_true, u_meas, z_gps = base.simulate(T=80.0, dt=dt, seed=7)

    # Start with under-estimated noises
    p = base.NoiseParams(sigma_v=0.02, sigma_w=0.008, sigma_gps=0.25, q_scale=0.5, r_scale=0.5)
    tgt = Targets()

    for it in range(6):
        _, nis_list, nees_list = base.run_filter_and_collect_stats(p, x_true, u_meas, z_gps, dt)
        p = tune_scales(p, nis_list, nees_list, tgt)
        print(f"Iter {it}: q_scale={p.q_scale:.3f}, r_scale={p.r_scale:.3f}, mean NIS={np.mean(nis_list):.3f}, mean NEES={np.mean(nees_list):.3f}")

    print("Tuned:", p)


if __name__ == "__main__":
    main()
