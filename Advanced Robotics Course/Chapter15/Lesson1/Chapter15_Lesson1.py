import numpy as np

# Parameters
N = 20          # number of agents
d = 2           # dimension
a = 0.5
b = 1.0
eps = 1e-2
R = 1.0         # sensing radius
h = 0.02        # time step
steps = 1000

# Initialize positions randomly in a square
np.random.seed(0)
x = np.random.uniform(low=-2.0, high=2.0, size=(N, d))

def phi_prime(r):
    # derivative of phi(r) = a r^2 - b log(r + eps)
    return 2.0 * a * r - b / (r + eps)

for k in range(steps):
    x_new = x.copy()
    for i in range(N):
        force = np.zeros(d)
        for j in range(N):
            if i == j:
                continue
            diff = x[i] - x[j]
            dist = np.linalg.norm(diff)
            if dist < 1e-6 or dist > R:
                continue
            fmag = phi_prime(dist)
            force += fmag * diff / dist
        x_new[i] = x[i] - h * force
    x = x_new

# Optionally visualize (requires matplotlib)
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    plt.scatter(x[:,0], x[:,1])
    plt.title("Final swarm configuration")
    plt.axis("equal")
    plt.show()
      
