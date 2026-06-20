import numpy as np

# Requirement vector r (accuracy, mobility, payload, safety)
r = np.array([0.7, 0.8, 0.5, 0.9])

classes = ["IndustrialArm", "MobileBase", "MobileManipulator", "Humanoid"]

# Capability matrix C (rows correspond to classes)
C = np.array([
    [0.95, 0.20, 0.90, 0.60],
    [0.30, 0.90, 0.40, 0.70],
    [0.80, 0.85, 0.70, 0.85],
    [0.75, 0.70, 0.55, 0.90]
])

# 1) Feasibility
feasible = [i for i in range(len(classes)) if np.all(C[i] >= r)]
print("Feasible classes:", [classes[i] for i in feasible])

# 2) Pareto-optimal among feasible
def dominates(ci, cj):
    return np.all(ci >= cj) and np.any(ci > cj)

pareto = []
for i in feasible:
    if not any(dominates(C[j], C[i]) for j in feasible if j != i):
        pareto.append(i)
print("Pareto set:", [classes[i] for i in pareto])

# 3) Utility
w = np.array([0.35, 0.25, 0.15, 0.25])  # weights sum to 1
alpha = np.array([4, 4, 3, 6])          # soft margin slopes
beta  = np.array([6, 5, 4, 8])          # penalty slopes
cost  = np.array([0.7, 0.4, 0.6, 0.9])  # normalized cost proxy

def satisfaction(c, r, alpha, beta):
    s = np.zeros_like(c)
    for j in range(len(c)):
        if c[j] >= r[j]:
            s[j] = 1 - np.exp(-alpha[j]*(c[j]-r[j]))
        else:
            s[j] = -beta[j]*(r[j]-c[j])
    return s

lam = 0.5  # cost importance
U = []
for i in range(len(classes)):
    s = satisfaction(C[i], r, alpha, beta)
    U.append(w @ s - lam*cost[i])

best = int(np.argmax(U))
print("Utility scores:", dict(zip(classes, U)))
print("Chosen class:", classes[best])
      