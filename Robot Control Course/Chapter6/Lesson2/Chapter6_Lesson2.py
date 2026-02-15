
import numpy as np

class HybridPositionForceController:
    """
    Hybrid position/force controller for a 3D point interacting with a planar surface.
    Position is controlled in the tangent plane; force is controlled along the normal.
    """

    def __init__(self, robot, contact_normal_world,
                 Kp_tan=None, Kd_tan=None, kf=50.0):
        self.robot = robot
        n = np.asarray(contact_normal_world, dtype=float).reshape(3)
        self.n = n / np.linalg.norm(n)
        self.S_f = np.outer(self.n, self.n)          # force-controlled projector
        self.S_p = np.eye(3) - self.S_f              # position-controlled projector

        if Kp_tan is None:
            # Default gains: only tangent directions matter (S_p will zero out normal)
            self.Kp_tan = 200.0 * np.eye(3)
        else:
            self.Kp_tan = np.asarray(Kp_tan, dtype=float)

        if Kd_tan is None:
            self.Kd_tan = 40.0 * np.eye(3)
        else:
            self.Kd_tan = np.asarray(Kd_tan, dtype=float)

        self.kf = float(kf)

    def compute_torque(self, q, qdot, x_d, xdot_d, Fd_n):
        """
        q, qdot: joint position/velocity (shape: (n,))
        x_d, xdot_d: desired position/velocity in R^3 (end-effector)
        Fd_n: desired normal force (scalar, positive when pushing along +n)
        """
        q = np.asarray(q, dtype=float).ravel()
        qdot = np.asarray(qdot, dtype=float).ravel()
        x_d = np.asarray(x_d, dtype=float).reshape(3)
        xdot_d = np.asarray(xdot_d, dtype=float).reshape(3)

        # Forward kinematics and Jacobian
        x = self.robot.forward_kinematics(q)         # shape (3,)
        J = self.robot.jacobian(q)                   # shape (3, n)
        xdot = J @ qdot

        # Project desired position onto tangent plane (avoid inconsistency)
        # x_d_proj = x_d - (n^T (x_d - x)) * n
        delta = x_d - x
        normal_component = (self.n @ delta) * self.n
        x_d_proj = x_d - normal_component

        # Position error (tangent only)
        e_p = self.S_p @ (x_d_proj - x)
        edot_p = self.S_p @ (xdot_d - xdot)

        # Tangential wrench command
        F_t_cmd = self.S_p @ (self.Kp_tan @ e_p + self.Kd_tan @ edot_p)

        # Measured force and normal component
        F_meas = self.robot.force_sensor().reshape(3)
        F_n = float(self.n @ F_meas)
        e_f = Fd_n - F_n

        # Normal wrench command (scalar times normal direction)
        F_n_cmd = Fd_n + self.kf * e_f
        F_n_vec = F_n_cmd * self.n

        # Total command wrench
        F_cmd = F_t_cmd + F_n_vec

        # Map to joint torques and add bias (inverse dynamics with zero qddot)
        tau = J.T @ F_cmd
        tau_bias = self.robot.inverse_dynamics(q, qdot, np.zeros_like(q))
        tau_total = tau + tau_bias
        return tau_total
