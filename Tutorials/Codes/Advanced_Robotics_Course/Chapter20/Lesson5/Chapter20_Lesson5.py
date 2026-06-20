import numpy as np
from typing import Callable, Dict, Any

# Assume env has methods: reset(task), step(u), is_done(), get_metrics()
# and policy has method act(obs)

def run_episode(env, policy, task_config: Dict[str, Any], max_steps: int = 500):
    obs = env.reset(task_config)
    success = 0
    metrics_accum = []
    for t in range(max_steps):
        u = policy.act(obs)
        obs, reward, done, info = env.step(u)
        metrics_accum.append(info.get("step_cost", 0.0))
        if done:
            success = 1 if info.get("success", False) else 0
            break
    episode_time = info.get("elapsed_time", t + 1)
    energy = info.get("energy_used", float(np.sum(metrics_accum)))
    return {"success": success, "time": episode_time, "energy": energy}

def evaluate_policy(env, policy, tasks: Callable[[int], Dict[str, Any]],
                    n_trials: int = 100, alpha: float = 0.05):
    successes = []
    times = []
    energies = []
    for i in range(n_trials):
        cfg = tasks(i)
        result = run_episode(env, policy, cfg)
        successes.append(result["success"])
        times.append(result["time"])
        energies.append(result["energy"])

    successes = np.asarray(successes, dtype=float)
    times = np.asarray(times, dtype=float)
    energies = np.asarray(energies, dtype=float)

    p_hat = successes.mean()
    # Wald-type CI for success rate
    z = 1.96  # for 95% confidence
    se_p = np.sqrt(p_hat * (1.0 - p_hat) / len(successes))
    ci_low = p_hat - z * se_p
    ci_high = p_hat + z * se_p

    return {
        "p_hat": p_hat,
        "p_ci": (max(0.0, ci_low), min(1.0, ci_high)),
        "time_mean": times.mean(),
        "time_std": times.std(ddof=1),
        "energy_mean": energies.mean(),
        "energy_std": energies.std(ddof=1),
    }

if __name__ == "__main__":
    # env, policy, and tasks(...) must be provided by your project
    stats = evaluate_policy(env, policy, tasks, n_trials=50)
    print("Success rate:", stats["p_hat"], "CI95:", stats["p_ci"])
    print("Mean time:", stats["time_mean"], "+/-", stats["time_std"])
    print("Mean energy:", stats["energy_mean"], "+/-", stats["energy_std"])
      
