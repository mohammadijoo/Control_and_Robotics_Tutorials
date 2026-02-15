import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# Define a small family of linear systems (toy "robots")
N_TASKS = 5
a_values = np.linspace(0.8, 1.1, N_TASKS)  # slightly unstable to stable
b = 0.5
lam = 0.1
gamma = 0.99

HORIZON = 50
N_EPISODES = 256

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def simulate_task(a, k, n_episodes=64):
    """Simulate a single specialist linear feedback policy u = k * x."""
    returns = []
    for _ in range(n_episodes):
        x = np.random.randn()  # random initial state
        G = 0.0
        disc = 1.0
        for _ in range(HORIZON):
            u = k * x
            r = -(x**2 + lam * u**2)
            G += disc * r
            disc *= gamma
            w = 0.01 * np.random.randn()
            x = a * x + b * u + w
        returns.append(G)
    return np.mean(returns)

# Simple specialist tuning via grid search over k for each task
def tune_specialist(a):
    ks = np.linspace(-5.0, 0.0, 51)
    best_k, best_ret = 0.0, -np.inf
    for k in ks:
        R = simulate_task(a, k, n_episodes=32)
        if R > best_ret:
            best_ret, best_k = R, k
    return best_k, best_ret

specialist_ks = []
specialist_returns = []
for a in a_values:
    k_opt, R_opt = tune_specialist(a)
    specialist_ks.append(k_opt)
    specialist_returns.append(R_opt)

print("Specialist ks:", specialist_ks)
print("Specialist avg return:", np.mean(specialist_returns))

# Generalist policy: shared network with task embedding
class GeneralistPolicy(nn.Module):
    def __init__(self, n_tasks, emb_dim=4, hidden=32):
        super().__init__()
        self.task_emb = nn.Embedding(n_tasks, emb_dim)
        self.net = nn.Sequential(
            nn.Linear(1 + emb_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, 1)
        )

    def forward(self, x, task_id):
        """
        x: [batch, 1] tensor of states
        task_id: [batch] tensor of ints in [0, n_tasks)
        returns: [batch, 1] actions
        """
        e = self.task_emb(task_id)
        inp = torch.cat([x, e], dim=-1)
        return self.net(inp)

policy = GeneralistPolicy(N_TASKS).to(device)
optimizer = optim.Adam(policy.parameters(), lr=3e-3)

def rollout_batch(policy, n_episodes_per_task=8):
    """Roll out the generalist on all tasks, returning Monte Carlo loss."""
    total_loss = 0.0
    for i, a in enumerate(a_values):
        for _ in range(n_episodes_per_task):
            x = torch.randn((), device=device)
            G = 0.0
            disc = 1.0
            for _ in range(HORIZON):
                task_id = torch.tensor([i], device=device, dtype=torch.long)
                x_tensor = x.view(1, 1)
                u = policy(x_tensor, task_id)[0, 0]
                r = -(x**2 + lam * u**2)
                G = G + disc * r
                disc *= gamma
                w = 0.01 * torch.randn((), device=device)
                x = a * x + b * u + w
            # We minimize negative return
            total_loss = total_loss - G
    return total_loss / (N_TASKS * n_episodes_per_task)

for it in range(300):
    optimizer.zero_grad()
    loss = rollout_batch(policy)
    loss.backward()
    optimizer.step()
    if (it + 1) % 50 == 0:
        print(f"Iter {it+1}, loss {loss.item():.3f}")

# Evaluate generalist performance on each task
def eval_generalist(policy, a, task_index, n_episodes=64):
    with torch.no_grad():
        rets = []
        for _ in range(n_episodes):
            x = torch.randn((), device=device)
            G = 0.0
            disc = 1.0
            for _ in range(HORIZON):
                tid = torch.tensor([task_index], device=device)
                u = policy(x.view(1, 1), tid)[0, 0]
                r = -(x**2 + lam * u**2)
                G = G + disc * r
                disc *= gamma
                w = 0.01 * torch.randn((), device=device)
                x = a * x + b * u + w
            rets.append(G.item())
        return np.mean(rets)

gen_returns = []
for i, a in enumerate(a_values):
    Rg = eval_generalist(policy, a, i)
    gen_returns.append(Rg)

print("Generalist per-task returns:", gen_returns)
print("Generalist avg return:", np.mean(gen_returns))
      
