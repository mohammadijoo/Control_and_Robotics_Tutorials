import numpy as np
import cvxpy as cp

def signed_distance_point_circle(q, center, radius):
    """
    Signed distance from point q to a circle with given center and radius.
    Positive outside, zero on boundary, negative inside.
    """
    return np.linalg.norm(q - center) - radius

def distance_linearization(q, center, radius):
    """
    Linear approximation of the signed distance at q:
      d(q + dq) approx d(q) + grad_d(q)^T dq
    For a circle, grad_d(q) = (q - center) / ||q - center||.
    """
    diff = q - center
    norm = np.linalg.norm(diff)
    if norm < 1e-8:
        # Avoid division by zero; use arbitrary unit vector
        grad = np.array([1.0, 0.0])
        d = -radius
    else:
        grad = diff / norm
        d = norm - radius
    return d, grad

def trajopt_sco(q_start, q_goal, obstacles, N=20,
                d_min=0.1, lambda_smooth=1.0,
                max_iters=10, trust_radius=0.3):
    """
    Simple SCO TrajOpt for a 2D point robot.
    obstacles: list of (center, radius) pairs.
    """
    # Initial guess: straight line interpolation
    qs = np.linspace(0.0, 1.0, N+1)[:, None]
    traj = (1 - qs) * q_start[None, :] + qs * q_goal[None, :]

    for it in range(max_iters):
        Q = cp.Variable((N+1, 2))

        # Smoothness cost (discrete acceleration)
        smooth_terms = []
        for k in range(1, N):
            smooth_terms.append(cp.sum_squares(Q[k+1] - 2*Q[k] + Q[k-1]))
        smooth_cost = lambda_smooth * cp.sum(smooth_terms)

        # Goal term
        goal_cost = cp.sum_squares(Q[N] - q_goal)

        # Collision constraints via linearization
        constraints = []
        constraints.append(Q[0] == q_start)
        constraints.append(Q[N] == q_goal)

        # Trust region around current trajectory
        constraints.append(cp.norm_inf(Q - traj) <= trust_radius)

        # Linearized signed distance constraints
        for k in range(N+1):
            qk = traj[k]
            for (center, radius) in obstacles:
                d, grad = distance_linearization(qk, center, radius)
                # d(qk) + grad^T (q - qk) >= d_min
                constraints.append(d + grad @ (Q[k] - qk) >= d_min)

        prob = cp.Problem(cp.Minimize(smooth_cost + goal_cost), constraints)
        prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)

        if prob.status not in ["optimal", "optimal_inaccurate"]:
            print("Iteration", it, ": QP not solved to optimality.")
            break

        new_traj = Q.value
        diff = np.linalg.norm(new_traj - traj)
        print(f"Iteration {it}, step norm = {diff:.4f}")
        traj = new_traj
        if diff < 1e-3:
            break

    return traj

if __name__ == "__main__":
    q_start = np.array([0.0, 0.0])
    q_goal = np.array([1.0, 1.0])
    obstacles = [
        (np.array([0.5, 0.5]), 0.2),
    ]
    traj = trajopt_sco(q_start, q_goal, obstacles)
    print("Final trajectory shape:", traj.shape)
      
