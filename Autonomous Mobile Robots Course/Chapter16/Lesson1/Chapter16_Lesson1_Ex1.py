"""
Chapter16_Lesson1_Ex1.py
Exercise scaffold: implement global assignment (Hungarian) instead of greedy NN.

Task:
- Given tracks and measurements, build a cost matrix using Mahalanobis distance.
- Apply gating: costs for gated-out pairs should be a large number.
- Solve assignment to minimize total cost (one-to-one).
- Return matches, unmatched tracks, unmatched measurements.

Hint:
- Use scipy.optimize.linear_sum_assignment if available.
"""

from __future__ import annotations
import numpy as np


def associate_hungarian(cost: np.ndarray, gate_mask: np.ndarray) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    """
    cost: shape (N_tracks, M_meas) with Mahalanobis distances (or d^2)
    gate_mask: same shape, True where pair is allowed (inside gate)

    Return:
      matches: list of (track_index, meas_index)
      unmatched_tracks: list of track indices
      unmatched_meas: list of meas indices
    """
    # TODO: implement
    raise NotImplementedError("Implement Hungarian assignment with gating.")
