
import numpy as np

class AdaptiveCTControllerPython:
    def __init__(self, n_dof, Kd, Lambda, Gamma_d, robot_model):
        """
        n_dof      : number of joints
        Kd         : (n_dof x n_dof) positive definite damping matrix
        Lambda     : (n_dof x n_dof) positive definite filtering matrix
        Gamma_d    : (p x p) discrete adaptation gain matrix
        robot_model: object providing regressor Y(q, dq, qr, dqr, ddqr)
        """
        self.n = n_dof
        self.Kd = Kd
        self.Lambda = Lambda
        self.Gamma_d = Gamma_d
        # p = number of base parameters of the robot
        self.robot_model = robot_model
        self.theta_hat = np.zeros(robot_model.num_params)

    def step(self, q, dq, qd, dqd, ddqd, dt):
        # tracking error
        e = q - qd
        de = dq - dqd
        # filtered error
        s = de + self.Lambda @ e
        # reference motion
        dqr = dqd - self.Lambda @ e
        ddqr = ddqd - self.Lambda @ de
        # regressor matrix
        Y = self.robot_model.regressor(q, dq, dqr, ddqr)  # shape (n, p)
        # control torque (use current theta_hat)
        tau = Y @ self.theta_hat - self.Kd @ s
        # parameter update (discrete form)
        grad = Y.T @ s
        self.theta_hat = self.theta_hat - self.Gamma_d @ grad

        return tau, self.theta_hat
