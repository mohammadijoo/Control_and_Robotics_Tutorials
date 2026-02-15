import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# -------------------------------------------------------
# 1. Load demonstrations: states S (N x d), actions A (N x m)
# -------------------------------------------------------
data = np.load("manip_demos.npz")
S = data["states"].astype(np.float32)   # shape (N, d)
A = data["actions"].astype(np.float32)  # shape (N, m)

N, d = S.shape
m = A.shape[1]

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

states = torch.from_numpy(S).to(device)
actions = torch.from_numpy(A).to(device)

# -------------------------------------------------------
# 2. Define neural policy pi_theta(s)
# -------------------------------------------------------
class MLPPolicy(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_sizes=(128, 128)):
        super().__init__()
        layers = []
        last_dim = state_dim
        for h in hidden_sizes:
            layers.append(nn.Linear(last_dim, h))
            layers.append(nn.ReLU())
            last_dim = h
        layers.append(nn.Linear(last_dim, action_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, s):
        return self.net(s)

policy = MLPPolicy(d, m).to(device)

# -------------------------------------------------------
# 3. Optimization setup
# -------------------------------------------------------
optimizer = optim.Adam(policy.parameters(), lr=1e-3, weight_decay=1e-4)
loss_fn = nn.MSELoss()

batch_size = 256
num_epochs = 50

# -------------------------------------------------------
# 4. Training loop
# -------------------------------------------------------
perm = torch.randperm(N)
states = states[perm]
actions = actions[perm]

for epoch in range(num_epochs):
    epoch_loss = 0.0
    for start in range(0, N, batch_size):
        end = min(start + batch_size, N)
        batch_s = states[start:end]
        batch_a = actions[start:end]

        pred_a = policy(batch_s)
        loss = loss_fn(pred_a, batch_a)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        epoch_loss += loss.item() * (end - start)

    epoch_loss /= N
    print(f"Epoch {epoch+1:03d} | loss = {epoch_loss:.6f}")

# -------------------------------------------------------
# 5. Example rollout in a robotics environment (pseudo-code)
# -------------------------------------------------------
# import pybullet as p
# env = MyManipEnv()
# state = env.reset()
# for t in range(max_steps):
#     s_vec = torch.from_numpy(state.astype(np.float32)).unsqueeze(0).to(device)
#     with torch.no_grad():
#         a_vec = policy(s_vec).cpu().numpy()[0]
#     state, obs = env.apply_velocity_command(a_vec)
#     env.render()
      
