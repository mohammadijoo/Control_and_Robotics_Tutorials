import torch
import torch.nn as nn
import torch.optim as optim

# Simple fully connected world model: (z_t, a_t) -> predicted z_{t+1}
class WorldModel(nn.Module):
    def __init__(self, state_dim=2, action_dim=1, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, state_dim)
        )

    def forward(self, z_t, a_t):
        inp = torch.cat([z_t, a_t], dim=-1)
        return self.net(inp)

# Generate synthetic data from known physics for supervised training
def generate_trajectories(num_traj=256, horizon=50, dt=0.05):
    z_list = []
    a_list = []
    z_next_list = []

    for _ in range(num_traj):
        x = torch.zeros(1)
        v = torch.zeros(1)
        for t in range(horizon):
            a = torch.randn(1) * 0.5
            x_next = x + dt * v
            v_next = v + dt * a

            z = torch.stack([x, v], dim=-1)          # shape (1, 2)
            z_next = torch.stack([x_next, v_next], dim=-1)
            z_list.append(z)
            a_list.append(a.view(1, 1))
            z_next_list.append(z_next)

            x, v = x_next, v_next

    z = torch.cat(z_list, dim=0)
    a = torch.cat(a_list, dim=0)
    z_next = torch.cat(z_next_list, dim=0)
    return z, a, z_next

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
wm = WorldModel().to(device)
optimizer = optim.Adam(wm.parameters(), lr=1e-3)

z, a, z_next = generate_trajectories()
dataset = torch.utils.data.TensorDataset(z, a, z_next)
loader = torch.utils.data.DataLoader(dataset, batch_size=128, shuffle=True)

for epoch in range(50):
    total_loss = 0.0
    for z_batch, a_batch, z_next_batch in loader:
        z_batch = z_batch.to(device)
        a_batch = a_batch.to(device)
        z_next_batch = z_next_batch.to(device)

        pred = wm(z_batch, a_batch)
        loss = ((pred - z_next_batch) ** 2).mean()

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * z_batch.size(0)

    total_loss /= len(dataset)
    if epoch % 10 == 0:
        print("epoch", epoch, "mse", total_loss)

# Imagination rollout from an initial latent state under a fixed policy
def imagine_rollout(wm, z0, policy, horizon=30):
    z_traj = [z0]
    a_traj = []
    for t in range(horizon):
        with torch.no_grad():
            z_t = z_traj[-1]
            a_t = policy(z_t)
            z_next = wm(z_t, a_t)
        a_traj.append(a_t)
        z_traj.append(z_next)
    return torch.stack(z_traj, dim=0), torch.stack(a_traj, dim=0)

# Simple proportional policy: accelerate toward target position x_star
def proportional_policy(x_star=1.0, k_p=1.0):
    def policy(z_t):
        x = z_t[:, 0:1]
        error = x_star - x
        a = k_p * error
        return a
    return policy

z0 = torch.tensor([[0.0, 0.0]], device=device)
policy = proportional_policy()
z_traj, a_traj = imagine_rollout(wm, z0, policy, horizon=40)

print("imagined final position", z_traj[-1, 0].item())
      
