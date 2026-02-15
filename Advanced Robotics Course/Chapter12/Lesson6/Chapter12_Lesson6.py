import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data

class PlanarReachEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 60}

    def __init__(self, render=False, dynamics_params=None):
        super().__init__()
        self.render = render
        self.dt = 1.0 / 60.0
        self.max_steps = 200
        self.step_count = 0

        # Dynamics parameters (mass, friction, etc.)
        default_params = {
            "link_masses": [1.0, 1.0],
            "joint_damping": [0.05, 0.05],
            "joint_max_torque": 5.0,
        }
        if dynamics_params is None:
            dynamics_params = default_params
        self.params = dynamics_params

        # Observation: [q1, q2, dq1, dq2, ee_x - goal_x, ee_y - goal_y]
        high = np.array([np.pi, np.pi, 5.0, 5.0, 2.0, 2.0], dtype=np.float32)
        self.observation_space = gym.spaces.Box(-high, high, dtype=np.float32)

        # Actions: joint torques
        max_torque = self.params["joint_max_torque"]
        self.action_space = gym.spaces.Box(
            low=-max_torque, high=max_torque, shape=(2,), dtype=np.float32
        )

        self._client = None
        self.robot_id = None
        self.goal = np.array([0.4, 0.4], dtype=np.float32)

    def _connect(self):
        if self._client is None:
            self._client = p.connect(p.GUI if self.render else p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81, physicsClientId=self._client)

    def _reset_sim(self):
        p.resetSimulation(physicsClientId=self._client)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client)
        plane_id = p.loadURDF("plane.urdf")
        # Simple 2-link arm URDF with revolute joints
        self.robot_id = p.loadURDF("two_link_manip.urdf", basePosition=[0, 0, 0])

        # Set dynamics parameters
        for link_idx, m in enumerate(self.params["link_masses"]):
            p.changeDynamics(
                self.robot_id, link_idx,
                mass=float(m),
                linearDamping=self.params["joint_damping"][link_idx],
                angularDamping=self.params["joint_damping"][link_idx],
            )

    def _get_state(self):
        joint_states = [
            p.getJointState(self.robot_id, j, physicsClientId=self._client)
            for j in range(2)
        ]
        q = np.array([js[0] for js in joint_states], dtype=np.float32)
        dq = np.array([js[1] for js in joint_states], dtype=np.float32)

        # EE position via forward kinematics from PyBullet
        ee_state = p.getLinkState(self.robot_id, 2, computeForwardKinematics=True)
        ee_pos = np.array(ee_state[0][0:2], dtype=np.float32)  # (x, y)
        diff = ee_pos - self.goal
        return np.concatenate([q, dq, diff], axis=0)

    def compute_reward(self, state, action):
        pos_error = state[-2:]
        torque_penalty = 0.001 * np.sum(action * action)
        return -float(pos_error.dot(pos_error) + torque_penalty)

    def step(self, action):
        self.step_count += 1
        # Clip action to torque limits
        action = np.clip(action, self.action_space.low, self.action_space.high)

        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=[0, 1],
            controlMode=p.TORQUE_CONTROL,
            forces=action,
            physicsClientId=self._client,
        )
        p.stepSimulation(physicsClientId=self._client)

        obs = self._get_state()
        reward = self.compute_reward(obs, action)
        terminated = False
        if np.linalg.norm(obs[-2:]) < 0.02:
            terminated = True
        truncated = self.step_count >= self.max_steps
        info = {}
        return obs, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._connect()
        self._reset_sim()
        self.step_count = 0

        # Randomize initial joint angles
        for j in range(2):
            init_q = self.np_random.uniform(low=-np.pi / 2.0, high=np.pi / 2.0)
            p.resetJointState(self.robot_id, j, init_q, 0.0, physicsClientId=self._client)

        obs = self._get_state()
        info = {}
        return obs, info

    def close(self):
        if self._client is not None:
            p.disconnect(physicsClientId=self._client)
            self._client = None
      
