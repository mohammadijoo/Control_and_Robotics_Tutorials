# Chapter15_Lesson3.py
# Finite-horizon observability Gramian and qualitative sensor placement.
#
# Libraries:
#   numpy: numerical arrays and eigenvalue routines.
# Optional extensions:
#   scipy.linalg.solve_continuous_lyapunov for infinite-horizon stable systems;
#   python-control for state-space model handling.

from __future__ import annotations

import itertools
import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


def gramian_rhs(A: np.ndarray, W: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """Right-hand side dW/dt = A.T W + W A + C.T C."""
    return A.T @ W + W @ A + Q


def finite_horizon_observability_gramian(
    A: np.ndarray,
    C: np.ndarray,
    T: float = 6.0,
    steps: int = 4000,
) -> np.ndarray:
    """
    Compute W_o(0,T) = integral_0^T exp(A.T t) C.T C exp(A t) dt
    through the matrix differential equation.

    This implementation uses RK4 and does not require SciPy.
    """
    n = A.shape[0]
    W = np.zeros((n, n), dtype=float)
    Q = C.T @ C
    h = T / steps

    for _ in range(steps):
        k1 = gramian_rhs(A, W, Q)
        k2 = gramian_rhs(A, W + 0.5 * h * k1, Q)
        k3 = gramian_rhs(A, W + 0.5 * h * k2, Q)
        k4 = gramian_rhs(A, W + h * k3, Q)
        W = W + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    return 0.5 * (W + W.T)


def row_sensor(index: int, n: int) -> np.ndarray:
    """Return a 1-by-n sensor row measuring x_index."""
    c = np.zeros((1, n), dtype=float)
    c[0, index] = 1.0
    return c


def stack_sensors(sensor_indices: Sequence[int], n: int) -> np.ndarray:
    """Build a C matrix from selected coordinate sensors."""
    return np.vstack([row_sensor(i, n) for i in sensor_indices])


def gramian_metrics(W: np.ndarray, eps: float = 1e-10) -> dict:
    """
    Qualitative scalar summaries:
      trace  -> total output energy averaged over coordinate directions
      logdet -> volume of the output-energy ellipsoid
      lambda_min -> weakest observable direction
      cond   -> anisotropy / numerical sensitivity
    """
    eigvals = np.linalg.eigvalsh(W)
    eigvals_clipped = np.maximum(eigvals, eps)

    rank = int(np.sum(eigvals > 1e-8))
    cond = math.inf if eigvals[0] <= eps else float(eigvals[-1] / eigvals[0])
    logdet = float(np.sum(np.log(eigvals_clipped)))

    return {
        "trace": float(np.trace(W)),
        "logdet_eps": logdet,
        "lambda_min": float(eigvals[0]),
        "lambda_max": float(eigvals[-1]),
        "rank": rank,
        "condition": cond,
        "eigvals": eigvals,
    }


def score_sensor_sets(
    A: np.ndarray,
    candidates: Sequence[int],
    k: int,
    T: float = 6.0,
) -> List[Tuple[Tuple[int, ...], dict]]:
    """Exhaustively score all k-sensor coordinate choices."""
    n = A.shape[0]
    scored = []
    for sensor_set in itertools.combinations(candidates, k):
        C = stack_sensors(sensor_set, n)
        W = finite_horizon_observability_gramian(A, C, T=T)
        scored.append((sensor_set, gramian_metrics(W)))
    scored.sort(key=lambda item: item[1]["logdet_eps"], reverse=True)
    return scored


def greedy_logdet_selection(
    A: np.ndarray,
    candidates: Sequence[int],
    k: int,
    T: float = 6.0,
    regularization: float = 1e-8,
) -> List[int]:
    """
    Greedy qualitative placement rule:
    at each step, add the sensor that gives the largest regularized log-det.
    """
    n = A.shape[0]
    selected: List[int] = []
    remaining = list(candidates)

    for step in range(k):
        best_sensor = None
        best_score = -math.inf

        for sensor in remaining:
            trial = selected + [sensor]
            C_trial = stack_sensors(trial, n)
            W_trial = finite_horizon_observability_gramian(A, C_trial, T=T)
            eigvals = np.linalg.eigvalsh(W_trial + regularization * np.eye(n))
            score = float(np.sum(np.log(eigvals)))

            if score > best_score:
                best_score = score
                best_sensor = sensor

        selected.append(best_sensor)
        remaining.remove(best_sensor)
        print(f"Step {step + 1}: add sensor x{best_sensor + 1}; logdet={best_score:.6f}")

    return selected


def main() -> None:
    # A three-state coupled stable system.
    A = np.array(
        [
            [0.0, 1.0, 0.0],
            [-2.0, -0.45, 0.8],
            [0.0, -0.7, -1.25],
        ],
        dtype=float,
    )

    candidates = [0, 1, 2]
    T = 6.0

    print("Single-sensor qualitative scores:")
    for sensor in candidates:
        C = stack_sensors([sensor], A.shape[0])
        W = finite_horizon_observability_gramian(A, C, T=T)
        metrics = gramian_metrics(W)
        print(
            f"x{sensor + 1}: trace={metrics['trace']:.4f}, "
            f"logdet_eps={metrics['logdet_eps']:.4f}, "
            f"lambda_min={metrics['lambda_min']:.4e}, "
            f"rank={metrics['rank']}, cond={metrics['condition']:.4e}"
        )

    print("\nBest two-sensor sets by regularized log-det:")
    for sensor_set, metrics in score_sensor_sets(A, candidates, k=2, T=T):
        names = ", ".join(f"x{i + 1}" for i in sensor_set)
        print(
            f"{{{names}}}: trace={metrics['trace']:.4f}, "
            f"logdet_eps={metrics['logdet_eps']:.4f}, "
            f"lambda_min={metrics['lambda_min']:.4e}, "
            f"rank={metrics['rank']}, cond={metrics['condition']:.4e}"
        )

    print("\nGreedy two-sensor choice:")
    greedy = greedy_logdet_selection(A, candidates, k=2, T=T)
    print("Selected:", [f"x{i + 1}" for i in greedy])


if __name__ == "__main__":
    main()
