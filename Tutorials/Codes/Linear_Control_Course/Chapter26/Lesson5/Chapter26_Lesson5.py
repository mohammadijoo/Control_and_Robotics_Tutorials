import numpy as np
from scipy import signal

# Design parameters
Ts = 0.002   # 2 ms sampling for a fast servo loop
tau_f = 0.01 # filter time constant (10 ms)

# Continuous-time first-order low-pass: F(s) = 1 / (1 + tau_f s)
num_c = [1.0]
den_c = [tau_f, 1.0]

# Discretize using backward Euler (or 'gbt'/'bilinear' for higher fidelity)
system_d = signal.cont2discrete((num_c, den_c), Ts, method="gbt", alpha=1.0)
b_d, a_d = system_d[0].flatten(), system_d[1].flatten()

print("Discrete coefficients b:", b_d, "a:", a_d)

class FirstOrderLPF:
    def __init__(self, b, a):
        self.b0 = b[0]
        self.a1 = a[1]
        self.y1 = 0.0

    def filter(self, u):
        # y[k] = -a1*y[k-1] + b0*u[k]
        y = -self.a1 * self.y1 + self.b0 * u
        self.y1 = y
        return y

lpf = FirstOrderLPF(b_d, a_d)

# Example: filtering a noisy velocity measurement
def filter_velocity_stream(samples):
    return [lpf.filter(v) for v in samples]
