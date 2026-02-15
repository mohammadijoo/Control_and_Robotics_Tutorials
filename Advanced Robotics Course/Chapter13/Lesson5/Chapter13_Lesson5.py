import numpy as np

class RandomizationConfig:
    def __init__(self, mass_range=(0.8, 1.2), friction_range=(0.5, 1.5),
                 latency_range=(0.0, 0.04)):
        self.mass_range = mass_range
        self.friction_range = friction_range
        self.latency_range = latency_range

    def sample(self, rng: np.random.Generator) -> dict:
        return {
            "mass_scale": rng.uniform(*self.mass_range),
            "friction_scale": rng.uniform(*self.friction_range),
            "latency": rng.uniform(*self.latency_range),
        }

def run_episode_sim(env, policy, params, max_steps=200):
    obs = env.reset(randomized_params=params)
    total_reward = 0.0
    success = False
    for t in range(max_steps):
        action = policy.act(obs)
        obs, reward, done, info = env.step(action)
        total_reward += reward
        if info.get("is_success", False):
            success = True
        if done:
            break
    return total_reward, success

def run_episode_real(policy, params, max_steps=200):
    # Thin wrapper around low-level real-robot interface.
    # Must enforce safety checks and emergency stop externally.
    result = real_robot_rollout(policy=policy,
                                params=params,
                                max_steps=max_steps)
    return result["return"], result["success"]

def evaluate_transfer(env, policy, rand_cfg_eval, n_eval=50, seed=0):
    rng = np.random.default_rng(seed)
    sim_returns, sim_success = [], []
    real_returns, real_success = [], []

    for i in range(n_eval):
        params = rand_cfg_eval.sample(rng)

        # Simulation episode
        R_sim, S_sim = run_episode_sim(env, policy, params)
        sim_returns.append(R_sim)
        sim_success.append(1 if S_sim else 0)

        # Real or high-fidelity episode
        R_real, S_real = run_episode_real(policy, params)
        real_returns.append(R_real)
        real_success.append(1 if S_real else 0)

    sim_returns = np.asarray(sim_returns, dtype=float)
    sim_success = np.asarray(sim_success, dtype=float)
    real_returns = np.asarray(real_returns, dtype=float)
    real_success = np.asarray(real_success, dtype=float)

    def summarize(vec):
        mean = vec.mean()
        std = vec.std(ddof=1)
        return mean, std

    J_sim, std_sim = summarize(sim_returns)
    J_real, std_real = summarize(real_returns)

    p_sim, std_p_sim = summarize(sim_success)
    p_real, std_p_real = summarize(real_success)

    transfer_gap_return = J_real - J_sim
    transfer_gap_success = p_real - p_sim

    return {
        "J_sim": J_sim,
        "J_real": J_real,
        "std_sim": std_sim,
        "std_real": std_real,
        "p_sim": p_sim,
        "p_real": p_real,
        "std_p_sim": std_p_sim,
        "std_p_real": std_p_real,
        "gap_return": transfer_gap_return,
        "gap_success": transfer_gap_success,
    }

# Example usage (env and policy assumed to be constructed elsewhere)
if __name__ == "__main__":
    rand_eval = RandomizationConfig(
        mass_range=(0.9, 1.1),
        friction_range=(0.8, 1.2),
        latency_range=(0.0, 0.03),
    )
    stats = evaluate_transfer(env, policy, rand_eval, n_eval=40, seed=42)
    print("Sim vs Real expected return:", stats["J_sim"], stats["J_real"])
    print("Sim vs Real success prob.:", stats["p_sim"], stats["p_real"])
    print("Transfer gaps:", stats["gap_return"], stats["gap_success"])
      
