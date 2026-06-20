import numpy as np

# Candidate sensors described by specs
# Each row: [range_min, range_max, fs, Delta, sigma_v, cost]
S = np.array([
    [0.0, 5.0, 200.0, 0.01, 0.02, 50.0],   # sensor A
    [0.1, 10.0, 60.0,  0.05, 0.05, 20.0],  # sensor B
    [0.0, 8.0, 120.0, 0.02, 0.03, 35.0],   # sensor C
])

# Task requirements
x_min_req, x_max_req = 0.0, 6.0
B = 40.0               # required bandwidth (Hz)
delta_x = 0.03         # required resolution
eps_max = 0.05         # max std dev allowed

def feasible(sensor):
    rmin, rmax, fs, Delta, sigma_v, cost = sensor
    # hard constraints
    if not (rmin <= x_min_req and rmax >= x_max_req):
        return False
    if fs < 2*B:
        return False
    if Delta > delta_x:
        return False
    sigma_tot = np.sqrt(sigma_v**2 + Delta**2/12)
    if sigma_tot > eps_max:
        return False
    return True

feasible_idx = [i for i,s in enumerate(S) if feasible(s)]
Sf = S[feasible_idx]

# Normalize attributes for scoring (lower is better)
# Attributes: sigma_tot, latency=1/fs, cost
sigma_tot = np.sqrt(Sf[:,4]**2 + Sf[:,3]**2/12)
latency = 1.0/Sf[:,2]
cost = Sf[:,5]

def normalize(v):
    return (v - v.min())/(v.max()-v.min() + 1e-12)

A = np.stack([normalize(sigma_tot), normalize(latency), normalize(cost)], axis=1)

w = np.array([0.5, 0.3, 0.2])  # weights sum to 1
scores = A @ w

best_local = np.argmin(scores)
best_global = feasible_idx[best_local]

print("Feasible sensors:", feasible_idx)
print("Scores:", scores)
print("Best sensor index:", best_global)
