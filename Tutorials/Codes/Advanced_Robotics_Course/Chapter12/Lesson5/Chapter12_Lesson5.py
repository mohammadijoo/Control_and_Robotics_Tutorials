import torch
import torch.nn as nn
import torch.optim as optim
import gymnasium as gym

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

env = gym.make("Pendulum-v1")  # stand-in for a torque-limited joint
obs_dim = env.observation_space.shape[0]
act_dim = env.action_space.shape[0]

# --- Actor-Critic with separate value heads for reward and constraint ---

class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.body = nn.Sequential(
            nn.Linear(obs_dim, 64), nn.Tanh(),
            nn.Linear(64, 64), nn.Tanh(),
        )
        self.mu_head = nn.Linear(64, act_dim)
        self.log_std = nn.Parameter(torch.zeros(act_dim))

        self.v_r = nn.Linear(64, 1)   # reward value
        self.v_c = nn.Linear(64, 1)   # constraint value

    def forward(self, obs):
        x = self.body(obs)
        mu = self.mu_head(x)
        std = self.log_std.exp()
        v_r = self.v_r(x)
        v_c = self.v_c(x)
        return mu, std, v_r, v_c

    def act(self, obs):
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device)
        mu, std, _, _ = self.forward(obs_t)
        dist = torch.distributions.Normal(mu, std)
        a = dist.sample()
        logp = dist.log_prob(a).sum(-1)
        return a.cpu().numpy(), logp

net = ActorCritic(obs_dim, act_dim).to(device)
optimizer = optim.Adam(net.parameters(), lr=3e-4)

lambda_c = torch.tensor(0.0, device=device, requires_grad=False)
lambda_lr = 1e-2
gamma = 0.99
cost_limit = 1.0

def compute_returns(rewards, costs, gamma):
    G_r, G_c = [], []
    g_r = 0.0
    g_c = 0.0
    for r, c in zip(reversed(rewards), reversed(costs)):
        g_r = r + gamma * g_r
        g_c = c + gamma * g_c
        G_r.append(g_r)
        G_c.append(g_c)
    G_r.reverse()
    G_c.reverse()
    return torch.tensor(G_r, dtype=torch.float32, device=device), \
           torch.tensor(G_c, dtype=torch.float32, device=device)

for epoch in range(1000):
    logps = []
    G_r_list = []
    G_c_list = []
    v_r_list = []
    v_c_list = []
    episode_returns = []
    episode_costs = []

    for _ in range(10):  # batch of episodes
        obs, _ = env.reset()
        done = False
        rewards = []
        costs = []
        logp_traj = []
        vals_r = []
        vals_c = []
        while not done:
            # define simple safety cost: penalty if position outside safe band
            x = obs[0]
            cost = 1.0 if abs(x) > 1.0 else 0.0

            action, logp = net.act(obs)
            next_obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            with torch.no_grad():
                obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device)
                _, _, v_r, v_c = net(obs_t.unsqueeze(0))
            vals_r.append(v_r.squeeze(0))
            vals_c.append(v_c.squeeze(0))

            rewards.append(float(reward))
            costs.append(float(cost))
            logp_traj.append(logp)
            obs = next_obs

        G_r, G_c = compute_returns(rewards, costs, gamma)
        logps.append(torch.stack(logp_traj))
        G_r_list.append(G_r)
        G_c_list.append(G_c)
        v_r_list.append(torch.stack(vals_r))
        v_c_list.append(torch.stack(vals_c))
        episode_returns.append(sum(rewards))
        episode_costs.append(sum(costs))

    logps = torch.cat(logps)
    G_r_list = torch.cat(G_r_list)
    G_c_list = torch.cat(G_c_list)
    v_r_list = torch.cat(v_r_list)
    v_c_list = torch.cat(v_c_list)

    # Advantage estimates (simple baseline)
    adv_r = G_r_list - v_r_list.detach()
    adv_c = G_c_list - v_c_list.detach()

    # Policy loss: maximize L = E[adv_r - lambda_c * adv_c]
    # (gradient ascent → minimize negative)
    loss_policy = -(logps * (adv_r - lambda_c * adv_c)).mean()

    # Value losses
    loss_v_r = 0.5 * (G_r_list - v_r_list).pow(2).mean()
    loss_v_c = 0.5 * (G_c_list - v_c_list).pow(2).mean()

    loss = loss_policy + loss_v_r + loss_v_c

    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

    # Dual update: push lambda_c up when cost exceeds limit
    avg_episode_cost = sum(episode_costs) / len(episode_costs)
    lambda_c = torch.clamp(lambda_c + lambda_lr *
                           (avg_episode_cost - cost_limit), min=0.0)

    print(f"Epoch {epoch}, return {sum(episode_returns)/len(episode_returns):.2f}, "
          f"avg cost {avg_episode_cost:.2f}, lambda {lambda_c.item():.3f}")
      
