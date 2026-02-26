import numpy as np

class CollectiveTransport2D:
    def __init__(self, N, kp=2.0, kv=0.5, alpha=1.0, beta=1.0, mass=10.0, dt=0.01):
        self.N = N
        self.kp = kp
        self.kv = kv
        self.alpha = alpha
        self.beta = beta
        self.m = mass
        self.dt = dt

        # Object state
        self.c = np.zeros(2)      # position
        self.v = np.zeros(2)      # velocity
        self.goal = np.array([1.0, 0.0])

        # Robot forces along a common direction d
        self.f = np.ones(N) * (1.0 / N)

        # Simple ring communication graph
        self.L = self._ring_laplacian(N)

    def _ring_laplacian(self, N):
        L = np.zeros((N, N))
        for i in range(N):
            L[i, i] = 2.0
            L[i, (i - 1) % N] = -1.0
            L[i, (i + 1) % N] = -1.0
        return L

    def step(self):
        # Object-level PD control
        e = self.c - self.goal
        Fd = -self.kp * e - self.kv * self.v
        normFd = np.linalg.norm(Fd)
        if normFd < 1e-6:
            return  # already at goal
        d = Fd / normFd
        Fd_mag = normFd

        # Distributed force allocation
        g = self.f - (Fd_mag / self.N) * np.ones(self.N)
        g_dot = -self.alpha * self.L.dot(g) - self.beta * g
        self.f += g_dot * self.dt

        # Enforce non-negativity and simple saturation
        self.f = np.clip(self.f, 0.0, 5.0)

        # Net force on object
        F_net = d * np.sum(self.f)

        # Update object state (simple Euler integration)
        a = F_net / self.m
        self.v += a * self.dt
        self.c += self.v * self.dt

if __name__ == "__main__":
    sim = CollectiveTransport2D(N=6)
    for k in range(1000):
        sim.step()
        if np.linalg.norm(sim.c - sim.goal) < 1e-2:
            break
    print("Final position:", sim.c)
      
