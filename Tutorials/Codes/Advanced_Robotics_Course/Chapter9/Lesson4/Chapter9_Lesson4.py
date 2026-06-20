import numpy as np
from scipy.optimize import minimize

# Robot and environment constants
dt = 0.1
T = 20                      # number of steps
q_start = np.array([0.0, 0.0])
q_goal  = np.array([1.0, 1.0])
obs_center = np.array([0.5, 0.5])
obs_radius = 0.2

def obstacle_penalty(q):
    # soft penalty if inside obstacle
    d = np.linalg.norm(q - obs_center) - obs_radius
    return max(0.0, -d)**2

def smoothness_cost(traj):
    # sum of squared velocities
    q = traj.reshape(T+1, 2)
    vel = np.diff(q, axis=0) / dt
    return np.sum(vel**2)

def goal_cost(traj):
    q = traj.reshape(T+1, 2)
    return np.linalg.norm(q[-1] - q_goal)**2

def collision_cost(traj):
    q = traj.reshape(T+1, 2)
    return sum(obstacle_penalty(q[k]) for k in range(T+1))

def make_push_cost(weight_push):
    def cost(traj):
        # weight_push penalizes deviation between object and robot
        q = traj.reshape(T+1, 2)
        obj = np.copy(q)          # object is pushed by contact with robot
        # encouraging contact: distance between robot and object small
        contact_pen = sum(np.linalg.norm(q[k] - obj[k])**2 for k in range(T+1))
        return smoothness_cost(traj) + goal_cost(traj) \
               + 10.0 * collision_cost(traj) \
               + weight_push * contact_pen
    return cost

def optimize_skeleton(mode):
    # mode can be "push" or "pick_place"
    if mode == "push":
        cost_fun = make_push_cost(weight_push=0.1)
    elif mode == "pick_place":
        # approximated by stronger contact requirement and a "lift" penalty
        cost_fun = make_push_cost(weight_push=10.0)
    else:
        raise ValueError("unknown mode")

    # decision variables: flattened trajectory (T+1 points in R^2)
    x0 = np.linspace(q_start, q_goal, T+1).reshape(-1)  # initial guess

    def eq_start_end(traj):
        q = traj.reshape(T+1, 2)
        return np.concatenate([q[0] - q_start])

    constraints = ({
        "type": "eq",
        "fun": eq_start_end,
    },)

    res = minimize(cost_fun, x0, method="SLSQP", constraints=constraints)
    return res

modes = ["push", "pick_place"]
best_val = np.inf
best_mode = None
best_traj = None
for mode in modes:
    res = optimize_skeleton(mode)
    if res.success and res.fun < best_val:
        best_val = res.fun
        best_mode = mode
        best_traj = res.x.reshape(T+1, 2)

print("Best skeleton:", best_mode, "cost:", best_val)
      
