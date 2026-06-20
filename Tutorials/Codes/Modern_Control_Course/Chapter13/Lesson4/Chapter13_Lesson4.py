# Chapter13_Lesson4.py
# Relationship of Observability to Sensor Placement
# Enumerates candidate state sensors and ranks their observability matrices.
#
# Recommended scientific stack:
#   pip install numpy scipy control
# This file uses only NumPy so it can run in a minimal environment.

import itertools
import numpy as np


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return O = [C; C A; ...; C A^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def observability_score(A: np.ndarray, C: np.ndarray, tol: float = 1e-9):
    """Return rank, smallest singular value, and log-det proxy."""
    O = observability_matrix(A, C)
    singular_values = np.linalg.svd(O, compute_uv=False)
    rank = int(np.sum(singular_values > tol))
    gram = O.T @ O
    eigvals = np.linalg.eigvalsh(gram + tol * np.eye(A.shape[0]))
    logdet = float(np.sum(np.log(eigvals)))
    sigma_min = float(singular_values[-1])
    return rank, sigma_min, logdet, O


def build_C_from_state_sensors(sensor_indices, n):
    """Each selected sensor measures one state directly: y_i = x_j."""
    rows = []
    for j in sensor_indices:
        row = np.zeros(n)
        row[j] = 1.0
        rows.append(row)
    return np.vstack(rows)


def enumerate_sensor_sets(A, max_sensors=None):
    """Enumerate direct state-sensor subsets and rank each placement."""
    n = A.shape[0]
    max_sensors = n if max_sensors is None else max_sensors
    results = []
    for r in range(1, max_sensors + 1):
        for subset in itertools.combinations(range(n), r):
            C = build_C_from_state_sensors(subset, n)
            rank, sigma_min, logdet, _ = observability_score(A, C)
            results.append({
                "sensors": tuple(j + 1 for j in subset),
                "rank": rank,
                "sigma_min": sigma_min,
                "logdet": logdet
            })
    results.sort(key=lambda item: (item["rank"], item["sigma_min"], item["logdet"]), reverse=True)
    return results


def main():
    # Four-state cascade-like model.
    # Measuring x1 can reveal downstream dynamic coupling differently from measuring x4.
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [-2.0, -0.4, 0.8, 0.0],
        [0.0, 0.0, -1.0, 1.0],
        [0.6, 0.0, -3.0, -0.5]
    ])

    print("A =")
    print(A)
    print("\nBest direct state-sensor placements:")
    for item in enumerate_sensor_sets(A, max_sensors=2)[:10]:
        print(
            f"sensors={item['sensors']}, "
            f"rank={item['rank']}, "
            f"sigma_min={item['sigma_min']:.3e}, "
            f"logdet={item['logdet']:.3f}"
        )

    # Inspect one placement in detail.
    C = build_C_from_state_sensors([0], A.shape[0])
    rank, sigma_min, logdet, O = observability_score(A, C)
    print("\nPlacement: measure x1")
    print("O =")
    print(O)
    print(f"rank={rank}, sigma_min={sigma_min:.3e}, logdet={logdet:.3f}")


if __name__ == "__main__":
    main()
