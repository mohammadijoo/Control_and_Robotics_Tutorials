import numpy as np

class Agent:
    def __init__(self, position, velocity, radius, v_pref, max_speed):
        self.p = np.asarray(position, dtype=float)
        self.v = np.asarray(velocity, dtype=float)
        self.r = float(radius)
        self.v_pref = np.asarray(v_pref, dtype=float)
        self.max_speed = float(max_speed)

def time_to_collision(p_ij, v_ij, R, T_h=np.inf):
    # Solve ||p_ij + t v_ij||^2 <= R^2
    a = np.dot(v_ij, v_ij)
    c = np.dot(p_ij, p_ij) - R**2
    if c <= 0.0:
        # Already in collision (overlap)
        return 0.0
    if a == 0.0:
        return np.inf  # No relative motion
    b = 2.0 * np.dot(p_ij, v_ij)
    disc = b*b - 4.0*a*c
    if disc < 0.0:
        return np.inf
    t_min = (-b - np.sqrt(disc)) / (2.0*a)
    if t_min < 0.0 or t_min > T_h:
        return np.inf
    return t_min

def normalize(v):
    n = np.linalg.norm(v)
    if n == 0.0:
        return v
    return v / n

def orca_constraints_for_agent(i, agents, T_h=5.0, delta=0.1):
    """
    Compute ORCA half-planes for agent i.
    Each constraint is (n_ij, p_point) meaning:
        (v - p_point) . n_ij >= 0
    """
    ai = agents[i]
    constraints = []

    for j, aj in enumerate(agents):
        if j == i:
            continue
        p_ij = aj.p - ai.p
        v_ij = ai.v - aj.v
        R = ai.r + aj.r
        ttc = time_to_collision(p_ij, v_ij, R, T_h)
        if ttc == np.inf:
            continue  # No impending collision

        # Compute closest point on VO boundary
        # Approximate by projecting relative velocity onto
        # tangent circle at time t = ttc
        p_coll = p_ij + v_ij * ttc
        n_ij = normalize(p_coll)  # outward normal of disk at collision
        # Small epsilon shift to guarantee separation
        u_ij = (R * n_ij - p_coll) / max(ttc, delta)

        # Agent i assumes half the correction
        p_point = ai.v + 0.5 * u_ij
        constraints.append((n_ij, p_point))

    return constraints

def project_velocity(v_pref, constraints, max_speed):
    """
    Solve min ||v - v_pref||^2 s.t. (v - p_k) . n_k >= 0, ||v|| <= max_speed.
    Simple incremental projection onto violated half-planes.
    """
    v = v_pref.copy()

    # Speed clipping (ball constraint)
    nrm = np.linalg.norm(v)
    if nrm > max_speed:
        v *= max_speed / nrm

    for (n_k, p_k) in constraints:
        if np.dot(v - p_k, n_k) < 0.0:
            # Project v onto boundary of half-plane
            # Solve (v' - p_k) . n_k = 0
            v = v + (np.dot(p_k - v, n_k)) * n_k

            # Re-enforce speed constraint
            nrm = np.linalg.norm(v)
            if nrm > max_speed:
                v *= max_speed / nrm

    return v

def step_orca(agents, T_h=5.0, delta=0.1):
    new_vels = []
    for i, _ in enumerate(agents):
        cons = orca_constraints_for_agent(i, agents, T_h, delta)
        v_star = project_velocity(agents[i].v_pref, cons, agents[i].max_speed)
        new_vels.append(v_star)

    # Update all agents synchronously
    for agent, v_star in zip(agents, new_vels):
        agent.v = v_star
        agent.p = agent.p + agent.v * delta

# Example usage
if __name__ == "__main__":
    agents = [
        Agent(position=[-1.0, 0.0], velocity=[0.5, 0.0],
              radius=0.3, v_pref=[0.5, 0.0], max_speed=1.0),
        Agent(position=[1.0, 0.0], velocity=[-0.5, 0.0],
              radius=0.3, v_pref=[-0.5, 0.0], max_speed=1.0),
    ]

    for k in range(50):
        step_orca(agents, T_h=5.0, delta=0.1)
        print(f"Step {k}:",
              [f"p{i}={agents[i].p}, v{i}={agents[i].v}" for i in range(len(agents))])
      
