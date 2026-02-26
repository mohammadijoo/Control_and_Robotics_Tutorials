import numpy as np
from typing import Callable, Dict, Any, List

# Example: general evaluation loop for a policy on a set of Gym-like environments.
# You can wrap MuJoCo, PyBullet, Isaac Gym, or MoveIt-based simulations in this interface.

class TaskEnv:
    def __init__(self, env_id: str, make_fn: Callable[[], Any]):
        self.env_id = env_id
        self.make_fn = make_fn

    def new_env(self):
        return self.make_fn()

def evaluate_policy_on_tasks(policy_fn: Callable[[np.ndarray, Dict], np.ndarray],
                             tasks: List[TaskEnv],
                             episodes_per_task: int = 10,
                             max_horizon: int = 200,
                             success_threshold: float = 0.0) -> Dict[str, float]:
    """
    policy_fn(obs, info) -> action
    """
    total_episodes = 0
    total_success = 0
    total_cost = 0.0

    for task in tasks:
        for _ in range(episodes_per_task):
            env = task.new_env()
            obs, info = env.reset()
            done = False
            step = 0
            episode_cost = 0.0
            while not done and step < max_horizon:
                action = policy_fn(obs, info)
                obs, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated
                # Assume "cost" is negative reward
                episode_cost += -float(reward)
                step += 1

            success = float(info.get("is_success", reward >= success_threshold))
            total_success += success
            total_cost += episode_cost
            total_episodes += 1

    success_rate = total_success / max(total_episodes, 1)
    avg_cost = total_cost / max(total_episodes, 1)
    return {"success_rate": success_rate, "avg_cost": avg_cost}

# Example usage stub:
# tasks = [
#     TaskEnv("PickPlace-v1", lambda: gym.make("PickPlace-v1")),
#     TaskEnv("DrawerOpen-v1", lambda: gym.make("DrawerOpen-v1")),
# ]
# results = evaluate_policy_on_tasks(my_generalist_policy, tasks)
# print(results)
      
