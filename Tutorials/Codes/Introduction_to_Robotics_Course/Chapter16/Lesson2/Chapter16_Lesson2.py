import numpy as np

# Each concept: cost [k€], mass [kg], energy [Wh per mission], tracking_error [deg rms]
concepts = [
    {"name": "A_wheeled", "cost": 15.0, "mass": 40.0, "energy": 800.0, "tracking_error": 1.5},
    {"name": "B_tracked", "cost": 18.0, "mass": 50.0, "energy": 700.0, "tracking_error": 1.2},
    {"name": "C_legged",  "cost": 25.0, "mass": 55.0, "energy": 950.0, "tracking_error": 0.8},
]

# Constraints
MAX_MASS = 55.0    # kg
MAX_ENERGY = 1000  # Wh

def is_feasible(c):
    return (c["mass"] <= MAX_MASS) and (c["energy"] <= MAX_ENERGY)

feasible = [c for c in concepts if is_feasible(c)]

# Build arrays of objectives to minimize
costs   = np.array([c["cost"] for c in feasible])
masses  = np.array([c["mass"] for c in feasible])
energies = np.array([c["energy"] for c in feasible])
errors  = np.array([c["tracking_error"] for c in feasible])

def normalize_min_to_max(arr):
    # returns z where 1 is best (minimum) and 0 is worst (maximum)
    amin, amax = arr.min(), arr.max()
    if np.isclose(amax, amin):
        return np.ones_like(arr)
    return (amax - arr) / (amax - amin)

Z = np.vstack([
    normalize_min_to_max(costs),
    normalize_min_to_max(masses),
    normalize_min_to_max(energies),
    normalize_min_to_max(errors),
]).T  # shape: (N_feasible, m)

names = [c["name"] for c in feasible]

# Choose weights (cost, mass, energy, tracking_error)
w = np.array([0.3, 0.2, 0.2, 0.3])

utilities = Z @ w
best_idx = int(np.argmax(utilities))

print("Utilities:")
for i, name in enumerate(names):
    print(f"{name}: U = {utilities[i]:.3f}")

print("\nBest concept under current weights:", names[best_idx])

# Pareto front (discrete dominance for 4 objectives to minimize)
F = np.vstack([costs, masses, energies, errors]).T

def dominates(i, j):
    """Return True if concept i dominates concept j (for minimization)."""
    return np.all(F[i] <= F[j]) and np.any(F[i] < F[j])

pareto_indices = []
for i in range(len(F)):
    dominated = False
    for j in range(len(F)):
        if i != j and dominates(j, i):
            dominated = True
            break
    if not dominated:
        pareto_indices.append(i)

print("\nPareto-optimal concepts:")
for idx in pareto_indices:
    print(names[idx], "with objectives", F[idx])
      
