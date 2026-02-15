import numpy as np

dt = 0.05
A = np.array([[1.0, dt],
              [0.0, 1.0]])
B = np.array([[0.5 * dt**2],
              [dt]])

# Simple feedback (e.g., PD-like on position and velocity)
K = np.array([[-2.0, -1.0]])  # shape (1,2)

p_max = 5.0
v_max = 3.0

T = 200                 # horizon length
N_trials = 5000         # number of Monte Carlo trials
sigma_w = 0.05          # disturbance std

def simulate_trial():
    # Random initial position and velocity within a box
    x = np.array([
        np.random.uniform(-4.0, 4.0),
        np.random.uniform(-2.0, 2.0)
    ])
    for k in range(T):
        u = (K @ x.reshape(-1, 1)).item()
        w = sigma_w * np.random.randn(2)
        x = A @ x + (B.flatten() * u) + w
        if abs(x[0]) > p_max or abs(x[1]) > v_max:
            return 1  # failure
    return 0          # success

failures = sum(simulate_trial() for _ in range(N_trials))
p_hat = failures / N_trials
print(f"Empirical failure rate: {p_hat:.4f}")
      
