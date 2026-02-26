import os
import json
import random
from dataclasses import dataclass, asdict
from datetime import datetime

import numpy as np

# Optional robotics libraries
import gymnasium as gym          # modern Gym API
import pybullet_envs  # noqa: F401  # registers PyBullet envs with Gym

try:
    import torch
except ImportError:
    torch = None


@dataclass
class ExperimentConfig:
    env_id: str = "KukaBulletEnv-v0"
    policy_name: str = "scripted_push"
    num_episodes: int = 50
    max_steps: int = 200
    global_seed: int = 1234


def set_global_seeds(seed: int) -> None:
    """
    Set all relevant seeds to make simulation as deterministic as possible.
    """
    random.seed(seed)
    np.random.seed(seed)
    if torch is not None:
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)


def scripted_policy(observation) -> np.ndarray:
    """
    Placeholder for a deterministic policy using the observation.
    In practice, this might be a fixed MPC controller or a frozen neural network.
    """
    # Example: zero action for all joints
    # Replace with your controller logic.
    return np.zeros(7, dtype=np.float32)


def run_single_episode(env, max_steps: int) -> dict:
    obs, info = env.reset()
    total_reward = 0.0
    success = False

    for _ in range(max_steps):
        action = scripted_policy(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += float(reward)
        if info.get("is_success", False):
            success = True
        if terminated or truncated:
            break

    return {
        "total_reward": total_reward,
        "success": success,
    }


def run_experiment(cfg: ExperimentConfig, output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)

    # Save configuration for reproducibility
    with open(os.path.join(output_dir, "config.json"), "w") as f:
        json.dump(asdict(cfg), f, indent=2)

    set_global_seeds(cfg.global_seed)

    env = gym.make(cfg.env_id)
    # Some environments accept seed in reset
    env.reset(seed=cfg.global_seed)

    results = []
    for episode in range(cfg.num_episodes):
        # To get independent but reproducible episodes, you can offset the seed
        env.reset(seed=cfg.global_seed + episode)
        ep_res = run_single_episode(env, cfg.max_steps)
        ep_res["episode"] = episode
        results.append(ep_res)

    env.close()

    # Save raw metrics
    with open(os.path.join(output_dir, "results.json"), "w") as f:
        json.dump(results, f, indent=2)

    # Compute summary statistics
    rewards = np.array([r["total_reward"] for r in results], dtype=np.float64)
    successes = np.array([r["success"] for r in results], dtype=np.float64)

    summary = {
        "mean_reward": float(np.mean(rewards)),
        "std_reward": float(np.std(rewards, ddof=1)),
        "success_rate": float(np.mean(successes)),
    }
    with open(os.path.join(output_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)


if __name__ == "__main__":
    cfg = ExperimentConfig()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join("logs", f"{cfg.policy_name}_{timestamp}")
    run_experiment(cfg, out_dir)
      
