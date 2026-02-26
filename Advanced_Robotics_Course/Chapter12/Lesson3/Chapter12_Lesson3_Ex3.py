import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# Simple MLP networks for actor and critic
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, action_dim), nn.Tanh()
        )
        self.max_action = max_action

    def forward(self, state):
        return self.max_action * self.net(state)

class Critic(nn.Module):
    # Twin critics Q1 and Q2
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.q1 = nn.Sequential(
            nn.Linear(state_dim + action_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, 1)
        )
        self.q2 = nn.Sequential(
            nn.Linear(state_dim + action_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, 1)
        )

    def forward(self, state, action):
        x = torch.cat([state, action], dim=-1)
        q1 = self.q1(x)
        q2 = self.q2(x)
        return q1, q2

    def q1_only(self, state, action):
        x = torch.cat([state, action], dim=-1)
        return self.q1(x)

class ReplayBuffer:
    def __init__(self, max_size, state_dim, action_dim):
        self.max_size = max_size
        self.ptr = 0
        self.size = 0
        self.state = np.zeros((max_size, state_dim), dtype=np.float32)
        self.action = np.zeros((max_size, action_dim), dtype=np.float32)
        self.next_state = np.zeros((max_size, state_dim), dtype=np.float32)
        self.reward = np.zeros((max_size, 1), dtype=np.float32)
        self.done = np.zeros((max_size, 1), dtype=np.float32)

    def add(self, s, a, r, s_next, d):
        self.state[self.ptr] = s
        self.action[self.ptr] = a
        self.next_state[self.ptr] = s_next
        self.reward[self.ptr] = r
        self.done[self.ptr] = d
        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)

    def sample(self, batch_size):
        idx = np.random.randint(0, self.size, size=batch_size)
        return (
            torch.tensor(self.state[idx]),
            torch.tensor(self.action[idx]),
            torch.tensor(self.reward[idx]),
            torch.tensor(self.next_state[idx]),
            torch.tensor(self.done[idx])
        )

class TD3Agent:
    def __init__(self, state_dim, action_dim, max_action,
                 gamma=0.99, tau=0.005,
                 policy_noise=0.2, noise_clip=0.5,
                 policy_delay=2, lr=3e-4):
        self.actor = Actor(state_dim, action_dim, max_action)
        self.actor_target = Actor(state_dim, action_dim, max_action)
        self.actor_target.load_state_dict(self.actor.state_dict())

        self.critic = Critic(state_dim, action_dim)
        self.critic_target = Critic(state_dim, action_dim)
        self.critic_target.load_state_dict(self.critic.state_dict())

        self.actor_opt = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_opt = optim.Adam(self.critic.parameters(), lr=lr)

        self.gamma = gamma
        self.tau = tau
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip
        self.policy_delay = policy_delay
        self.total_it = 0
        self.max_action = max_action

    def select_action(self, state, noise_std=0.1):
        state_t = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            action = self.actor(state_t).cpu().numpy()[0]
        if noise_std is not None:
            action += np.random.normal(0.0, noise_std, size=action.shape)
        return np.clip(action, -self.max_action, self.max_action)

    def train(self, replay, batch_size=256):
        if replay.size < batch_size:
            return

        self.total_it += 1
        state, action, reward, next_state, done = replay.sample(batch_size)

        with torch.no_grad():
            # Target policy smoothing
            noise = torch.normal(0.0, self.policy_noise,
                                 size=action.shape)
            noise = torch.clamp(noise, -self.noise_clip, self.noise_clip)

            next_action = self.actor_target(next_state)
            next_action = torch.clamp(
                next_action + noise,
                -self.max_action,
                self.max_action
            )

            q1_targ, q2_targ = self.critic_target(next_state, next_action)
            q_min = torch.min(q1_targ, q2_targ)
            target_q = reward + self.gamma * (1.0 - done) * q_min

        # Critic update
        q1, q2 = self.critic(state, action)
        critic_loss = nn.MSELoss()(q1, target_q) + nn.MSELoss()(q2, target_q)

        self.critic_opt.zero_grad()
        critic_loss.backward()
        self.critic_opt.step()

        # Delayed policy update
        if self.total_it % self.policy_delay == 0:
            actor_loss = -self.critic.q1_only(state, self.actor(state)).mean()
            self.actor_opt.zero_grad()
            actor_loss.backward()
            self.actor_opt.step()

            # Polyak averaging for target networks
            with torch.no_grad():
                for p, p_targ in zip(self.critic.parameters(),
                                     self.critic_target.parameters()):
                    p_targ.data.mul_(1.0 - self.tau)
                    p_targ.data.add_(self.tau * p.data)
                for p, p_targ in zip(self.actor.parameters(),
                                     self.actor_target.parameters()):
                    p_targ.data.mul_(1.0 - self.tau)
                    p_targ.data.add_(self.tau * p.data)
      
