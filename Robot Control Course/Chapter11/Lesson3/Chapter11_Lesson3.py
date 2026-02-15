
import numpy as np

class RobotEKF:
    def __init__(self, n_dof, dt):
        self.n = n_dof
        self.dt = dt
        dim_x = 2 * n_dof

        self.x = np.zeros((dim_x, 1))  # [q; dq]
        self.P = np.eye(dim_x) * 1e-2

        # Process and measurement noise covariances
        q_pos = 1e-6
        q_vel = 1e-4
        self.Q = np.block([
            [q_pos * np.eye(n_dof), np.zeros((n_dof, n_dof))],
            [np.zeros((n_dof, n_dof)), q_vel * np.eye(n_dof)]
        ])

        # Example: joint encoder + ee position (3d) noise
        r_enc = 1e-5
        r_ee = 1e-4
        self.dim_z = n_dof + 3
        self.R = np.diag(
            np.concatenate([
                r_enc * np.ones(n_dof),
                r_ee * np.ones(3)
            ])
        )

    def f(self, x, u):
        """Nonlinear process model x_{k+1} = f(x_k, u_k)."""
        n = self.n
        q = x[0:n].reshape(-1, 1)
        dq = x[n:2*n].reshape(-1, 1)

        # Robot forward dynamics: qddot = M^{-1}(q) * (tau - C(q,dq)dq - g(q))
        # This must be implemented using your robot model.
        qddot = robot_forward_dynamics(q, dq, u)

        q_next = q + self.dt * dq
        dq_next = dq + self.dt * qddot
        return np.vstack((q_next, dq_next))

    def F_jacobian(self, x, u, eps=1e-6):
        """Numerical Jacobian of f wrt x."""
        dim_x = x.shape[0]
        F = np.zeros((dim_x, dim_x))
        fx = self.f(x, u)
        for i in range(dim_x):
            dx = np.zeros_like(x)
            dx[i, 0] = eps
            f_plus = self.f(x + dx, u)
            F[:, i:i+1] = (f_plus - fx) / eps
        return F

    def h(self, x):
        """Measurement model: stacked joint positions and ee position."""
        n = self.n
        q = x[0:n].reshape(-1, 1)
        # Encoders measure q directly
        z_q = q

        # Forward kinematics: ee_pos in R^3
        ee_pos = forward_kinematics(q)  # shape (3,1)
        return np.vstack((z_q, ee_pos))

    def H_jacobian(self, x, eps=1e-6):
        """Numerical Jacobian of h wrt x."""
        dim_x = x.shape[0]
        z0 = self.h(x)
        dim_z = z0.shape[0]
        H = np.zeros((dim_z, dim_x))
        for i in range(dim_x):
            dx = np.zeros_like(x)
            dx[i, 0] = eps
            H[:, i:i+1] = (self.h(x + dx) - z0) / eps
        return H

    def predict(self, u):
        """EKF prediction step."""
        x = self.x
        P = self.P

        # Mean prediction
        self.x = self.f(x, u)

        # Jacobian wrt x
        F = self.F_jacobian(x, u)
        self.P = F @ P @ F.T + self.Q

    def update(self, z):
        """EKF update step with measurement z."""
        x_pred = self.x
        P_pred = self.P

        z_pred = self.h(x_pred)
        H = self.H_jacobian(x_pred)

        y = z.reshape(-1, 1) - z_pred                      # innovation
        S = H @ P_pred @ H.T + self.R                     # innovation covariance
        K = P_pred @ H.T @ np.linalg.inv(S)               # Kalman gain

        self.x = x_pred + K @ y
        I = np.eye(P_pred.shape[0])
        self.P = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ self.R @ K.T

# Example placeholders for robot-specific functions:
def robot_forward_dynamics(q, dq, tau):
    # Implement using your dynamics library (Pinocchio, RBDL, etc.)
    # Here we simply use a dummy model with unit inertia and no coupling.
    return tau  # qddot = tau (very simplified)

def forward_kinematics(q):
    # Implement your forward kinematics to 3D ee position
    # Here, return a dummy linear function.
    return np.array([[q.sum()], [0.0], [0.0]])
