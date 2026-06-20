import numpy as np

class PlanarArmEnvDomainRandomized:
    def __init__(self,
                 mass_nominal=(1.0, 1.0),
                 mass_rel_range=0.3,
                 friction_nominal=0.05,
                 friction_rel_range=0.5,
                 seed=None):
        self.rng = np.random.default_rng(seed)
        self.mass_nominal = np.array(mass_nominal, dtype=float)
        self.mass_rel_range = mass_rel_range
        self.friction_nominal = float(friction_nominal)
        self.friction_rel_range = friction_rel_range

        # Parameters that will be randomized each reset
        self.link_masses = self.mass_nominal.copy()
        self.joint_friction = np.array([self.friction_nominal,
                                        self.friction_nominal], dtype=float)

        # State: [q1, q2, dq1, dq2]
        self.state = np.zeros(4, dtype=float)
        self.dt = 0.02

    def _sample_parameters(self):
        # Uniform multiplicative perturbation
        low_m = (1.0 - self.mass_rel_range) * self.mass_nominal
        high_m = (1.0 + self.mass_rel_range) * self.mass_nominal
        self.link_masses = self.rng.uniform(low_m, high_m)

        low_f = (1.0 - self.friction_rel_range) * self.friction_nominal
        high_f = (1.0 + self.friction_rel_range) * self.friction_nominal
        self.joint_friction[:] = self.rng.uniform(low_f, high_f, size=2)

    def reset(self):
        self._sample_parameters()
        # Small random initial state
        self.state = self.rng.normal(0.0, 0.05, size=4)
        return self.state.copy()

    def step(self, u):
        # u: torques (2-dim)
        u = np.clip(u, -5.0, 5.0)

        # Very simplified dynamics: dqdot = (u - friction * dq) / mass
        q1, q2, dq1, dq2 = self.state
        dq = np.array([dq1, dq2])
        # element-wise "effective mass" per joint for illustration
        eff_mass = self.link_masses + 1e-3
        ddq = (u - self.joint_friction * dq) / eff_mass

        dq_new = dq + self.dt * ddq
        q_new = np.array([q1, q2]) + self.dt * dq_new

        self.state = np.concatenate([q_new, dq_new])

        # Reward: negative distance to target plus small control penalty
        target = np.array([0.0, 0.0])
        pos_error = np.linalg.norm(q_new - target)
        r = -pos_error - 0.01 * np.dot(u, u)

        done = False  # finite horizon handled outside
        info = {
            "link_masses": self.link_masses.copy(),
            "joint_friction": self.joint_friction.copy()
        }
        return self.state.copy(), float(r), done, info
      
