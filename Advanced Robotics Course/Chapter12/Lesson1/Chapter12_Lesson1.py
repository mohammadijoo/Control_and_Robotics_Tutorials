import numpy as np

class ContinuousRobotMDP:
    """
    Minimal continuous-state, continuous-action MDP wrapper for a robot.

    State x = [q, dq] in R^{2n}
    Action u in R^{m} (e.g. joint torques)
    """

    def __init__(self, n_dof, m_act, dt=0.02):
        self.n = n_dof
        self.m = m_act
        self.dt = dt

        # Cost weights for quadratic running cost
        self.Q = np.eye(2 * n_dof)   # state cost
        self.R = 0.01 * np.eye(m_act)  # action cost

        # Simple joint limits and action limits
        self.q_limit = np.ones(n_dof) * np.pi
        self.u_limit = np.ones(m_act) * 10.0

    def dynamics(self, x, u):
        """
        Simple double-integrator style dynamics per joint:
          q_{t+1} = q_t + dt * dq_t
          dq_{t+1} = dq_t + dt * ddq_t
        where ddq_t approximates M^{-1} B u.
        """
        x = np.asarray(x, dtype=float)
        u = np.asarray(u, dtype=float)

        n = self.n
        q = x[:n]
        dq = x[n:]

        # Clip control to limits
        u_clipped = np.clip(u, -self.u_limit, self.u_limit)

        # Crude acceleration model (unit inertia)
        ddq = u_clipped  # in real robots, use full rigid-body dynamics

        q_next = q + self.dt * dq
        dq_next = dq + self.dt * ddq

        # Enforce simple joint limits
        q_next = np.clip(q_next, -self.q_limit, self.q_limit)

        x_next = np.concatenate([q_next, dq_next])
        return x_next

    def reward(self, x, u, x_des=None):
        """
        Quadratic tracking reward in state and control.
        For now, desired state x_des is the zero vector if not given.
        """
        x = np.asarray(x, dtype=float)
        u = np.asarray(u, dtype=float)

        if x_des is None:
            x_des = np.zeros_like(x)

        dx = x - x_des
        cost = dx.T @ self.Q @ dx + u.T @ self.R @ u
        return -float(cost)

    def step(self, x, u, x_des=None):
        """
        Apply one MDP transition:
          x_next ~ p(. | x, u)
        Here the dynamics are deterministic; stochasticity could be added
        by injecting Gaussian noise into x_next.
        """
        x_next = self.dynamics(x, u)
        r = self.reward(x, u, x_des)

        # Example termination condition: huge norm means failure
        done = bool(np.linalg.norm(x_next) > 1e3)

        return x_next, r, done

# Example usage:
if __name__ == "__main__":
    env = ContinuousRobotMDP(n_dof=2, m_act=2, dt=0.01)
    x = np.array([0.1, -0.1, 0.0, 0.0])  # [q1, q2, dq1, dq2]
    u = np.array([1.0, -0.5])
    x_next, r, done = env.step(x, u)
    print("x_next:", x_next)
    print("reward:", r, "done:", done)
      
