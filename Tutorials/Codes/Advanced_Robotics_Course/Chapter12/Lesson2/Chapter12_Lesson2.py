import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import gymnasium as gym

env = gym.make("Pendulum-v1")  # continuous torque control
obs_dim = env.observation_space.shape[0]
act_dim = env.action_space.shape[0]

class Actor(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, act_dim)
        )
        # log standard deviation (state-independent for simplicity)
        self.log_std = nn.Parameter(torch.zeros(act_dim))

    def forward(self, obs):
        mu = self.net(obs)
        std = torch.exp(self.log_std)
        return mu, std

    def sample(self, obs):
        mu, std = self.forward(obs)
        dist = torch.distributions.Normal(mu, std)
        a = dist.rsample()          # reparameterized sample
        logp = dist.log_prob(a).sum(axis=-1)
        # squash to [-2, 2] (Pendulum action bounds)
        squashed = 2.0 * torch.tanh(a)
        return squashed, logp

class Critic(nn.Module):
    def __init__(self, obs_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )

    def forward(self, obs):
        return self.net(obs).squeeze(-1)

actor = Actor(obs_dim, act_dim)
critic = Critic(obs_dim)
actor_opt = optim.Adam(actor.parameters(), lr=3e-4)
critic_opt = optim.Adam(critic.parameters(), lr=3e-3)
gamma = 0.99

def run_episode():
    obs, _ = env.reset()
    traj = []
    done = False
    while not done:
        obs_t = torch.as_tensor(obs, dtype=torch.float32)
        with torch.no_grad():
            a, logp = actor.sample(obs_t)
        a_np = a.numpy()
        next_obs, reward, terminated, truncated, _ = env.step(a_np)
        done = terminated or truncated
        traj.append((obs, a_np, reward, logp.item(), next_obs))
        obs = next_obs
    return traj

for episode in range(2000):
    traj = run_episode()

    # collect tensors
    obs = torch.as_tensor([t[0] for t in traj], dtype=torch.float32)
    logp = torch.as_tensor([t[3] for t in traj], dtype=torch.float32)
    rewards = [t[2] for t in traj]
    next_obs = torch.as_tensor([t[4] for t in traj], dtype=torch.float32)

    # compute returns and critic values
    returns = []
    G = 0.0
    for r in reversed(rewards):
        G = r + gamma * G
        returns.append(G)
    returns.reverse()
    returns = torch.as_tensor(returns, dtype=torch.float32)

    V = critic(obs)
    with torch.no_grad():
        V_next = critic(next_obs)
    # TD error as advantage
    r_t = torch.as_tensor(rewards, dtype=torch.float32)
    delta = r_t + gamma * V_next - V
    advantages = delta.detach()

    # critic loss (TD error squared)
    critic_loss = (delta ** 2).mean()
    critic_opt.zero_grad()
    critic_loss.backward()
    critic_opt.step()

    # actor loss: gradient ascent on J => minimize -E[log pi * advantage]
    actor_loss = -(logp * advantages).mean()
    actor_opt.zero_grad()
    actor_loss.backward()
    actor_opt.step()

    if episode % 100 == 0:
        print(f"Episode {episode}, return = {returns[0].item():.2f}")
      
