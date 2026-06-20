
import numpy as np

class JointFilter:
    def __init__(self, n_joints, Ts, omega_c_vel, omega_c_pos=None):
        self.n = n_joints
        self.Ts = Ts
        # Dirty derivative parameters
        self.alpha_vel = 1.0 / (1.0 + omega_c_vel * Ts)
        self.q_prev = np.zeros(n_joints)
        self.qf = np.zeros(n_joints)
        self.qdot_est = np.zeros(n_joints)
        if omega_c_pos is not None:
            self.beta_pos = 1.0 / (1.0 + omega_c_pos * Ts)
        else:
            self.beta_pos = None

    def update(self, q_meas):
        if self.beta_pos is None:
            self.qf = q_meas.copy()
        else:
            self.qf = self.beta_pos * self.qf + (1.0 - self.beta_pos) * q_meas

        dq_raw = (q_meas - self.q_prev) / self.Ts
        self.qdot_est = (self.alpha_vel * self.qdot_est
                         + (1.0 - self.alpha_vel) * dq_raw)
        self.q_prev = q_meas.copy()
        return self.qf, self.qdot_est


def saturate(u, u_min, u_max):
    return np.minimum(np.maximum(u, u_min), u_max)


def rate_limit(u_cmd, u_prev, du_min, du_max):
    du = u_cmd - u_prev
    du = np.minimum(np.maximum(du, du_min), du_max)
    return u_prev + du


class ComputedTorqueController:
    def __init__(self, robot_model, Kp, Kd, Ts,
                 tau_min, tau_max, dtau_min, dtau_max,
                 omega_c_vel, omega_c_pos=None):
        """
        robot_model: object with methods M(q), C(q, qdot), g(q)
        Kp, Kd: diagonal gain matrices (as np.array)
        tau_min, tau_max: joint torque bounds
        dtau_min, dtau_max: per-sample torque increment bounds
        """
        self.robot = robot_model
        self.Kp = Kp
        self.Kd = Kd
        self.Ts = Ts
        self.tau_min = tau_min
        self.tau_max = tau_max
        self.dtau_min = dtau_min
        self.dtau_max = dtau_max
        self.filter = JointFilter(len(tau_min), Ts,
                                  omega_c_vel, omega_c_pos)
        self.tau_prev = np.zeros_like(tau_min)

    def step(self, q_meas, qd, qd_dot, qd_ddot):
        # 1) Filtering
        qf, qdot_est = self.filter.update(q_meas)

        # 2) Errors
        e_q = qd - qf
        e_qdot = qd_dot - qdot_est

        # 3) Dynamics terms
        M = self.robot.M(qf)
        C = self.robot.C(qf, qdot_est)
        g = self.robot.g(qf)

        v = qd_ddot + self.Kd @ e_qdot + self.Kp @ e_q
        tau_pre = M @ v + C @ qdot_est + g

        # 4) Rate limiting then saturation
        tau_rl = rate_limit(tau_pre, self.tau_prev,
                            self.dtau_min, self.dtau_max)
        tau_sat = saturate(tau_rl, self.tau_min, self.tau_max)

        self.tau_prev = tau_sat.copy()
        return tau_sat
