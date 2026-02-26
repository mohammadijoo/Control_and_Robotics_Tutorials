
import numpy as np

class JointDOB:
    def __init__(self, J_nom, w_c, dt):
        self.J_nom = J_nom
        self.w_c = w_c
        self.dt = dt
        self.tau_u_hat = 0.0
        self.q_prev = 0.0
        self.qdot_prev = 0.0
        self.initialized = False

    def update(self, q, qdot, tau_c):
        """
        q     : current joint position
        qdot  : current joint velocity
        tau_c : commanded torque (outer-loop before DOB)
        returns: compensated torque tau = tau_c - tau_u_hat
        """
        if not self.initialized:
            self.q_prev = q
            self.qdot_prev = qdot
            self.initialized = True

        # Approximate acceleration from velocity (could be low-pass filtered in practice)
        qddot = (qdot - self.qdot_prev) / self.dt

        # Disturbance estimate dynamics (forward Euler discretization)
        rhs = -self.w_c * self.tau_u_hat + self.w_c * (self.J_nom * qddot - tau_c)
        self.tau_u_hat = self.tau_u_hat + self.dt * rhs

        tau = tau_c - self.tau_u_hat

        self.q_prev = q
        self.qdot_prev = qdot
        return tau

# Example use inside a control loop
dt = 0.001
dob = JointDOB(J_nom=1.0, w_c=50.0, dt=dt)
Kp, Kd = 100.0, 20.0

def joint_controller(q, qdot, q_des, qdot_des, qddot_des):
    # nominal computed-torque-like law
    tau_c = (1.0 * qddot_des
             + Kp * (q_des - q)
             + Kd * (qdot_des - qdot))
    # DOB compensation
    tau = dob.update(q, qdot, tau_c)
    return tau
