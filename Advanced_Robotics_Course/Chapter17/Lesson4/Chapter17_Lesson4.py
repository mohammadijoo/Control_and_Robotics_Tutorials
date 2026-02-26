import torch
import torch.nn as nn
import torch.optim as optim

# Hyperparameters
obs_dim = 128   # e.g., encoded deformable state
act_dim = 6     # e.g., Cartesian velocity (vx, vy, vz, wx, wy, wz)
hidden_dim = 256
lr = 1e-3

class DeformablePolicy(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, act_dim)
        )

    def forward(self, obs):
        return self.net(obs)

policy = DeformablePolicy(obs_dim, act_dim, hidden_dim)

# Dummy dataset: replace with real features and expert actions
# observations: Tensor of shape (N, obs_dim)
# actions_star: Tensor of shape (N, act_dim)
observations = torch.randn(1024, obs_dim)
actions_star = torch.randn(1024, act_dim)

dataset = torch.utils.data.TensorDataset(observations, actions_star)
loader = torch.utils.data.DataLoader(dataset, batch_size=64, shuffle=True)

optimizer = optim.Adam(policy.parameters(), lr=lr)
mse_loss = nn.MSELoss()

for epoch in range(50):
    running_loss = 0.0
    for obs_batch, act_batch in loader:
        optimizer.zero_grad()
        act_pred = policy(obs_batch)
        loss = mse_loss(act_pred, act_batch)
        loss.backward()
        optimizer.step()
        running_loss += loss.item() * obs_batch.size(0)
    avg_loss = running_loss / len(dataset)
    print(f"Epoch {epoch:03d}, Loss = {avg_loss:.6f}")

# Example of deploying the policy given a new observation feature vector:
with torch.no_grad():
    o_new = torch.randn(1, obs_dim)  # replace with real encoded observation
    u_cmd = policy(o_new)
    print("Commanded velocity:", u_cmd.cpu().numpy())
      
