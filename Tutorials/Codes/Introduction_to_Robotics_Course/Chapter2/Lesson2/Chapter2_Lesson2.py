
import numpy as np

def cubic_ptp(q0, qf, T, t):
    """Cubic point-to-point joint interpolation."""
    t = np.clip(t, 0.0, T)
    Delta = qf - q0
    s = t / T
    return q0 + 3*Delta*s**2 - 2*Delta*s**3

# Example usage
q0, qf, T = 0.2, 1.0, 2.0
ts = np.linspace(0, T, 50)
qs = cubic_ptp(q0, qf, T, ts)
print(qs[:5])
      