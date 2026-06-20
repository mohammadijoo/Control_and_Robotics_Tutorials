import numpy as np

class Swarm2D:
    def __init__(self, N=30, dt=0.05, behavior="aggregate",
                 d_des=0.2, domain=((0.0, 1.0), (0.0, 1.0))):
        self.N = N
        self.dt = dt
        self.behavior = behavior
        self.d_des = d_des
        self.domain = domain
        # positions: shape (N, 2)
        self.x = np.random.rand(N, 2)
        self.x[:, 0] = self.x[:, 0] * (domain[0][1] - domain[0][0]) + domain[0][0]
        self.x[:, 1] = self.x[:, 1] * (domain[1][1] - domain[1][0]) + domain[1][0]
        # parameters
        self.k_agg = 1.0
        self.k_disp = 0.5
        self.k_cov = 1.0
        self.R = 0.4  # sensing radius

    def neighbors(self, i):
        diffs = self.x - self.x[i]
        dists = np.linalg.norm(diffs, axis=1)
        idx = np.where((dists > 0.0) & (dists < self.R))[0]
        return idx, diffs[idx], dists[idx]

    def step_aggregate(self):
        u = np.zeros_like(self.x)
        for i in range(self.N):
            idx, diffs, _ = self.neighbors(i)
            if idx.size > 0:
                u[i] = -self.k_agg * np.sum(diffs, axis=0)
        self.x += self.dt * u

    def step_dispersion(self):
        u = np.zeros_like(self.x)
        for i in range(self.N):
            idx, diffs, dists = self.neighbors(i)
            if idx.size == 0:
                continue
            # Pairwise potential derivative (r^2 - d_des^2) r
            r2 = dists ** 2
            phi_prime = (r2 - self.d_des ** 2) * dists
            # unit vectors
            dirs = diffs / dists[:, None]
            # sum over neighbors
            force = -np.sum(phi_prime[:, None] * dirs, axis=0)
            u[i] = self.k_disp * force
        self.x += self.dt * u

    def step_coverage(self, M_samples=500):
        # Approximate Lloyd step: sample domain, assign to nearest robot,
        # compute sample-based centroids, move robots toward them.
        xs = np.random.rand(M_samples, 2)
        xs[:, 0] = xs[:, 0] * (self.domain[0][1] - self.domain[0][0]) + self.domain[0][0]
        xs[:, 1] = xs[:, 1] * (self.domain[1][1] - self.domain[1][0]) + self.domain[1][0]
        # density phi(q) can be non-uniform; here uniform
        # assign samples to closest robot
        dists = np.linalg.norm(xs[:, None, :] - self.x[None, :, :], axis=2)  # M x N
        closest = np.argmin(dists, axis=1)
        centroids = np.zeros_like(self.x)
        counts = np.zeros(self.N, dtype=int)
        for m in range(M_samples):
            i = closest[m]
            centroids[i] += xs[m]
            counts[i] += 1
        for i in range(self.N):
            if counts[i] > 0:
                c_i = centroids[i] / counts[i]
                # gradient step toward centroid
                self.x[i] += self.dt * self.k_cov * (c_i - self.x[i])

    def step(self):
        if self.behavior == "aggregate":
            self.step_aggregate()
        elif self.behavior == "disperse":
            self.step_dispersion()
        elif self.behavior == "cover":
            self.step_coverage()
        # Optional: keep robots inside domain (simple clipping)
        self.x[:, 0] = np.clip(self.x[:, 0], self.domain[0][0], self.domain[0][1])
        self.x[:, 1] = np.clip(self.x[:, 1], self.domain[1][0], self.domain[1][1])

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    swarm = Swarm2D(N=40, behavior="aggregate")
    for k in range(200):
        swarm.step()
        if k % 20 == 0:
            plt.clf()
            plt.scatter(swarm.x[:, 0], swarm.x[:, 1])
            plt.xlim(swarm.domain[0])
            plt.ylim(swarm.domain[1])
            plt.title(f"Step {k}")
            plt.pause(0.01)
    plt.show()
      
