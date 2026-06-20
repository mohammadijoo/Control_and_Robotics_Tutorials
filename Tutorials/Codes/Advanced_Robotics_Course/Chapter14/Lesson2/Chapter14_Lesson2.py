import numpy as np

# Number of agents
N = 4

# Complete graph Laplacian: L = N*I - 1*ones
L = N * np.eye(N) - np.ones((N, N))

# Step-size alpha satisfying 0 < alpha < 2 / lambda_max
alpha = 0.3  # for N = 4, 2 / lambda_max = 0.5

# Iteration matrix P
P = np.eye(N) - alpha * L

# Initial states (e.g., positions on a line)
x = np.array([0.0, 2.0, -1.0, 4.0])

def step_consensus(x):
    return P @ x

history = [x.copy()]
T = 30
for k in range(T):
    x = step_consensus(x)
    history.append(x.copy())

print("Final state:", x)
print("Average of initial states:", np.mean(history[0]))

# Example: plot trajectories (if matplotlib installed)
try:
    import matplotlib.pyplot as plt
    hist = np.array(history)
    for i in range(N):
        plt.plot(hist[:, i], label=f"agent {i+1}")
    plt.axhline(np.mean(history[0]), linestyle="--", label="average")
    plt.xlabel("k")
    plt.ylabel("x_i(k)")
    plt.legend()
    plt.show()
except ImportError:
    print("matplotlib not available; skipping plot.")
      
