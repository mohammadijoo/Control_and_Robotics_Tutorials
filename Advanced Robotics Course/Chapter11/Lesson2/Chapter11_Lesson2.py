import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# Simple planar arm state: [q1, q2, dq1, dq2]
STATE_DIM = 4
ACTION_DIM = 2   # joint torques tau1, tau2

class PolicyNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(STATE_DIM, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, ACTION_DIM)
        )

    def forward(self, x):
        return self.net(x)

def expert_policy(state):
    """
    Placeholder for an expert controller, e.g.,
    a PD controller around a reference trajectory.
    state: np.array of shape (4,)
    returns: np.array of shape (2,)
    """
    # Here we just implement a dummy proportional controller
    q_des = np.array([0.5, -0.5])
    q = state[:2]
    dq = state[2:]
    Kp = 5.0
    Kd = 1.0
    tau = Kp * (q_des - q) - Kd * dq
    return tau

def simulate_dynamics(state, action, dt=0.02):
    """
    Very rough placeholder dynamics integrator.
    In practice, use a robotics simulator (e.g. PyBullet, Mujoco).
    """
    q = state[:2]
    dq = state[2:]
    # Simple double integrator approximation: ddq = action
    ddq = action
    dq_next = dq + ddq * dt
    q_next = q + dq_next * dt
    return np.concatenate([q_next, dq_next], axis=0)

def collect_expert_trajectory(T=200):
    states = []
    actions = []
    state = np.zeros(STATE_DIM, dtype=np.float32)
    for t in range(T):
        a = expert_policy(state)
        states.append(state.copy())
        actions.append(a.astype(np.float32))
        state = simulate_dynamics(state, a)
    return np.array(states), np.array(actions)

def train_bc(policy, dataset, epochs=20, batch_size=64, lr=1e-3):
    opt = optim.Adam(policy.parameters(), lr=lr)
    criterion = nn.MSELoss()
    states, actions = dataset
    states_t = torch.from_numpy(states)
    actions_t = torch.from_numpy(actions)
    N = states.shape[0]
    for ep in range(epochs):
        perm = torch.randperm(N)
        for i in range(0, N, batch_size):
            idx = perm[i:i+batch_size]
            s_batch = states_t[idx]
            a_batch = actions_t[idx]
            opt.zero_grad()
            pred = policy(s_batch)
            loss = criterion(pred, a_batch)
            loss.backward()
            opt.step()

def rollout(policy, T=200):
    states = []
    state = np.zeros(STATE_DIM, dtype=np.float32)
    for t in range(T):
        with torch.no_grad():
            s_t = torch.from_numpy(state).float().unsqueeze(0)
            a_t = policy(s_t).squeeze(0).numpy()
        states.append(state.copy())
        state = simulate_dynamics(state, a_t)
    return np.array(states)

# --- Behavior Cloning (single-shot) ---
expert_states, expert_actions = collect_expert_trajectory(T=200)
policy = PolicyNet()
train_bc(policy, (expert_states, expert_actions), epochs=50)

# --- DAgger loop ---
def dagger_training(num_iters=5, T=200):
    policy = PolicyNet()
    # Initial dataset from expert-only rollouts
    states, actions = collect_expert_trajectory(T)
    for i in range(num_iters):
        # Train policy on current dataset
        train_bc(policy, (states, actions), epochs=20)
        # Rollout current policy to explore its own state distribution
        roll_states = rollout(policy, T)
        # Query expert at visited states
        new_actions = np.array([expert_policy(s) for s in roll_states], dtype=np.float32)
        # Aggregate dataset
        states = np.concatenate([states, roll_states], axis=0)
        actions = np.concatenate([actions, new_actions], axis=0)
        print(f"DAgger iter {i+1}, dataset size = {states.shape[0]}")
    return policy, (states, actions)

dagger_policy, agg_dataset = dagger_training()
      
