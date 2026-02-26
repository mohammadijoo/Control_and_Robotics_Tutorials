import numpy as np

# Simple 1D environment with long horizon
class LongHorizon1DEnv:
    def __init__(self, dt=0.1, horizon=200, x0=0.0, x_target=10.0,
                 process_std=0.01, obs_std=0.05):
        self.dt = dt
        self.H = horizon
        self.x_target = x_target
        self.proc_std = process_std
        self.obs_std = obs_std
        self.reset(x0)

    def reset(self, x0=0.0):
        self.t = 0
        self.x = float(x0)
        o = self.x + np.random.randn() * self.obs_std
        return o

    def step(self, u):
        # Dynamics
        w = np.random.randn() * self.proc_std
        self.x = self.x + self.dt * float(u) + w
        # Observation
        o = self.x + np.random.randn() * self.obs_std
        # Cost: squared position error + quadratic control
        pos_err = self.x - self.x_target
        cost = pos_err**2 + 0.01 * float(u)**2
        self.t += 1
        done = (self.t >= self.H)
        return o, -cost, done, {}

# Recurrent policy with explicit hidden state (memory)
class RNNPolicy:
    def __init__(self, obs_dim=1, hidden_dim=16, u_max=1.0):
        self.obs_dim = obs_dim
        self.hidden_dim = hidden_dim
        self.u_max = u_max
        # Parameters: simple tanh RNN + linear output
        rng = np.random.default_rng(0)
        self.Wxh = rng.normal(scale=0.1, size=(hidden_dim, obs_dim))
        self.Whh = rng.normal(scale=0.1, size=(hidden_dim, hidden_dim))
        self.bh = np.zeros((hidden_dim, 1))
        self.Why = rng.normal(scale=0.1, size=(1, hidden_dim))
        self.by = np.zeros((1, 1))
        self.h = np.zeros((hidden_dim, 1))  # memory state

    def reset_hidden(self):
        self.h[:] = 0.0

    def forward(self, o):
        # o: scalar observation
        obs = np.array([[float(o)]])
        self.h = np.tanh(self.Wxh @ obs + self.Whh @ self.h + self.bh)
        u = self.Why @ self.h + self.by
        # Squash to control limits
        u = np.clip(u, -self.u_max, self.u_max)
        return float(u)

# Rollout with fixed (untrained) parameters to illustrate memory usage
if __name__ == "__main__":
    env = LongHorizon1DEnv()
    policy = RNNPolicy()
    num_episodes = 3

    for ep in range(num_episodes):
        o = env.reset(x0=0.0)
        policy.reset_hidden()
        total_return = 0.0
        for t in range(env.H):
            u = policy.forward(o)
            o, r, done, info = env.step(u)
            total_return += r
            if done:
                break
        print(f"Episode {ep}, total return (untrained): {total_return:.2f}")
      
