import numpy as np

# Example parameter vector: [link_mass_scale, joint_damping_scale, friction_scale]
PARAM_DIM = 3

def sample_theta():
    # Simple box constraints: theta in [0.5, 1.5]^3
    return 0.5 + np.random.rand(PARAM_DIM)

def trajectory_features(traj):
    # traj: dict with keys "q", "qd", "ee_pos"
    q = traj["q"]      # shape: (T, nq)
    qd = traj["qd"]    # shape: (T, nq)
    ee = traj["ee_pos"]  # shape: (T, 3)
    # Simple feature: stack mean and std of each signal
    feats = []
    for arr in [q, qd, ee]:
        feats.append(arr.mean(axis=0))
        feats.append(arr.std(axis=0))
    return np.concatenate(feats, axis=0)

def feature_mismatch(real_traj, sim_traj, W=None):
    f_real = trajectory_features(real_traj)
    f_sim = trajectory_features(sim_traj)
    diff = f_real - f_sim
    if W is None:
        return float(diff @ diff)
    return float(diff @ (W @ diff))

def calibration_objective(theta, real_experiments, sim_interface, W=None):
    cost = 0.0
    for exp in real_experiments:
        # exp is a dict with "u", "real_traj", "init_state"
        sim_traj = sim_interface.rollout_sim(theta, exp)
        cost += feature_mismatch(exp["real_traj"], sim_traj, W=W)
    return cost

def random_search_calibration(real_experiments, sim_interface,
                              num_samples=100, seed=0):
    rng = np.random.default_rng(seed)
    best_theta = None
    best_cost = np.inf
    for k in range(num_samples):
        theta = sample_theta()
        cost = calibration_objective(theta, real_experiments, sim_interface)
        if cost < best_cost:
            best_cost = cost
            best_theta = theta
        if (k + 1) % 10 == 0:
            print(f"Iter {k+1}: best_cost = {best_cost:.3f}")
    return best_theta, best_cost

# Usage (assuming sim_interface and real_experiments are defined):
# theta_star, J_star = random_search_calibration(real_experiments, sim_interface)
# print("Calibrated theta:", theta_star)
      
