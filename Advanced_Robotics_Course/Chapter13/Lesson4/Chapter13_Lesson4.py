import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# Example dimensions for a 7-DOF manipulator (q, qdot)
n_q = 7
n_x = 2 * n_q
n_u = n_q
dt = 0.01

class ResidualNet(nn.Module):
    def __init__(self, state_dim, act_dim, hidden_dim=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + act_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, state_dim),
        )

    def forward(self, x, u):
        # x: (batch, n_x), u: (batch, n_u)
        inp = torch.cat([x, u], dim=-1)
        return self.net(inp)

def simulate_phys(x_np, u_np, dt):
    """
    Placeholder for a call to a robotics physics engine.
    x_np: np.array with shape (n_x,)
    u_np: np.array with shape (n_u,)
    Returns next state x_next_np.
    """
    # Here we just do identity plus small drift as a stand-in.
    # In practice, call your simulator or analytic dynamics integrator.
    return x_np + dt * np.concatenate([x_np[n_q:], np.zeros_like(x_np[n_q:])])

# Build dataset from real rollouts (collected on the real robot).
# real_data is assumed to be an array of triples (x_t, u_t, x_tp1_real).
real_data = []  # fill from experiments
# Example: real_data.append((x_t_np, u_t_np, x_tp1_real_np))

model = ResidualNet(n_x, n_u)
optimizer = optim.Adam(model.parameters(), lr=1e-3)
lambda_reg = 1e-3

def train_epoch(real_data, batch_size=64):
    np.random.shuffle(real_data)
    total_loss = 0.0
    for i in range(0, len(real_data), batch_size):
        batch = real_data[i:i + batch_size]
        x_list, u_list, x_next_real_list = zip(*batch)
        x = torch.tensor(np.stack(x_list), dtype=torch.float32)
        u = torch.tensor(np.stack(u_list), dtype=torch.float32)
        x_next_real = torch.tensor(np.stack(x_next_real_list), dtype=torch.float32)

        # Nominal physics prediction
        x_next_phys_list = []
        for j in range(x.shape[0]):
            x_np = x[j].detach().numpy()
            u_np = u[j].detach().numpy()
            x_next_phys_np = simulate_phys(x_np, u_np, dt)
            x_next_phys_list.append(x_next_phys_np)
        x_next_phys = torch.tensor(np.stack(x_next_phys_list), dtype=torch.float32)

        # Residual prediction
        r = model(x, u)
        x_next_hyb = x_next_phys + r

        loss_pred = ((x_next_hyb - x_next_real) ** 2).mean()
        loss_reg = (r ** 2).mean()
        loss = loss_pred + lambda_reg * loss_reg

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * x.shape[0]

    return total_loss / len(real_data)

# After training, use f_hyb(x, u) = simulate_phys(x, u, dt) + model(x, u)
      
