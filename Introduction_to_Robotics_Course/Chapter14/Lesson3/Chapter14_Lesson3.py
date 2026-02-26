import numpy as np

# In a real robot stack, these functions operate on logs from a ROS node
# that records target distances, widths, and timestamps.

def shannon_id(D, W):
    """Index of difficulty ID = log2(1 + D/W)."""
    D = np.asarray(D, dtype=float)
    W = np.asarray(W, dtype=float)
    return np.log2(1.0 + D / W)

def fitts_mt(D, W, a, b):
    ID = shannon_id(D, W)
    return a + b * ID

def throughput(ID, MT):
    """Throughput TP = sum(ID) / sum(MT)."""
    ID = np.asarray(ID, dtype=float)
    MT = np.asarray(MT, dtype=float)
    return ID.sum() / MT.sum()

def hick_rt(probs, a, b):
    """Reaction time RT = a + b * H, H = -sum p log2 p."""
    p = np.asarray(probs, dtype=float)
    p = p / p.sum()
    H = -(p * np.log2(p)).sum()
    return a + b * H

def safe_velocity(t_r, a_max, d_safe):
    """
    Solve v^2/(2 a_max) + t_r v - d_safe = 0 for v >= 0.
    Returns the smaller non-negative root (the safety-bound speed).
    """
    A = 1.0 / (2.0 * a_max)
    B = t_r
    C = -d_safe
    disc = B**2 - 4.0 * A * C
    if disc < 0.0:
        return 0.0
    v1 = (-B + np.sqrt(disc)) / (2.0 * A)
    v2 = (-B - np.sqrt(disc)) / (2.0 * A)
    candidates = [v for v in (v1, v2) if v >= 0.0]
    return min(candidates) if candidates else 0.0

if __name__ == "__main__":
    # Example: three pointing tasks
    D = np.array([0.20, 0.30, 0.40])  # meters
    W = np.array([0.04, 0.05, 0.06])  # meters
    MT = np.array([0.45, 0.50, 0.55])  # seconds
    ID = shannon_id(D, W)
    TP = throughput(ID, MT)
    print("Throughput [bits/s]:", TP)

    # Example: four commands with different probabilities
    probs = np.array([0.4, 0.3, 0.2, 0.1])
    rt = hick_rt(probs, a=0.25, b=0.15)
    print("Reaction time [s]:", rt)

    # Safe approach speed based on that reaction time
    a_max = 3.0   # m/s^2
    d_safe = 1.5  # m
    v_max = safe_velocity(rt, a_max, d_safe)
    print("Max safe approach speed [m/s]:", v_max)
      
