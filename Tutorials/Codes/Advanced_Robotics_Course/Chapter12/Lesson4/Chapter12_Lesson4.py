import numpy as np

# Base dense reward for a reaching task
def base_reward(ee_pos, goal, action, success, ctrl_coeff=0.01):
    dist = np.linalg.norm(ee_pos - goal)
    r_task = -dist
    r_ctrl = -ctrl_coeff * np.dot(action, action)
    r_term = 1.0 if success else 0.0
    return r_task + r_ctrl + r_term

# Potential-based shaping with Phi(s) = -alpha * ||ee_pos - goal||
def potential(ee_pos, goal, alpha=0.5):
    return -alpha * np.linalg.norm(ee_pos - goal)

def shaped_reward(prev_obs, obs, action, gamma=0.99, alpha=0.5):
    ee_prev = prev_obs["ee_pos"]
    goal = prev_obs["goal"]
    ee = obs["ee_pos"]
    success = obs["success"]

    r_base = base_reward(ee_prev, goal, action, success)
    phi_prev = potential(ee_prev, goal, alpha=alpha)
    phi = potential(ee, goal, alpha=alpha)
    r_shape = gamma * phi - phi_prev
    return r_base + r_shape

# Minimal HER-style relabelling for a batch of transitions
def her_relabel_episode(episode, k_future=4):
    """
    episode: list of dicts with keys
      "obs", "action", "next_obs", "reward", "done"
    """
    her_transitions = []
    T = len(episode)
    for t in range(T):
        transition = episode[t]
        obs = transition["obs"]
        next_obs = transition["next_obs"]

        # sample alternative goals from future states
        future_idxs = np.random.randint(t, T, size=k_future)
        for idx in future_idxs:
            alt_goal = episode[idx]["next_obs"]["ee_pos"].copy()
            obs_her = dict(obs)
            next_obs_her = dict(next_obs)
            obs_her["goal"] = alt_goal
            next_obs_her["goal"] = alt_goal

            # recompute success flag under alternative goal
            d = np.linalg.norm(next_obs_her["ee_pos"] - alt_goal)
            success = d <= 0.02
            next_obs_her["success"] = success

            r_her = base_reward(
                obs_her["ee_pos"], obs_her["goal"], transition["action"], success
            )
            her_transitions.append(
                {
                    "obs": obs_her,
                    "action": transition["action"],
                    "next_obs": next_obs_her,
                    "reward": r_her,
                    "done": transition["done"],
                }
            )
    return her_transitions
      
