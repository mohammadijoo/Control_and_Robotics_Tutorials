
import numpy as np
import cvxpy as cp

class JointCBFQP1DOF:
    def __init__(self, q_min, q_max,
                 gamma=5.0,
                 kp=20.0, kd=5.0,
                 u_min=-2.0, u_max=2.0):
        self.q_min = q_min
        self.q_max = q_max
        self.gamma = gamma
        self.kp = kp
        self.kd = kd
        self.u_min = u_min
        self.u_max = u_max

    def nominal_velocity(self, q, qdot, q_ref, qdot_ref=0.0):
        # Simple PD in velocity space (tracks q_ref)
        e = q_ref - q
        edot = qdot_ref - qdot
        return self.kp * e + self.kd * edot

    def cbf_bounds(self, q):
        # h_low = q - q_min, h_up = q_max - q
        h_low = q - self.q_min
        h_up = self.q_max - q

        # alpha(h) = gamma * h
        alpha_low = self.gamma * h_low
        alpha_up = self.gamma * h_up

        # CBF inequalities:
        # u >= -alpha_low, u <= alpha_up
        return -alpha_low, alpha_up

    def solve_qp(self, q, qdot, q_ref):
        u_nom = self.nominal_velocity(q, qdot, q_ref)

        u = cp.Variable()  # scalar joint velocity
        alpha_low, alpha_up = self.cbf_bounds(q)

        constraints = []
        # CBF constraints
        constraints += [u >= alpha_low]   # note: alpha_low = -gamma (q - q_min)
        constraints += [u <= alpha_up]
        # actuator limits
        constraints += [u >= self.u_min, u <= self.u_max]

        # Minimize (u - u_nom)^2
        objective = cp.Minimize(0.5 * cp.square(u - u_nom))
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP, warm_start=True)

        if u.value is None:
            # Fall back to saturated nominal if QP fails
            u_safe = np.clip(u_nom, self.u_min, self.u_max)
        else:
            u_safe = float(u.value)
        return u_safe

# Example usage in a simple integration loop
if __name__ == "__main__":
    dt = 0.01
    q_min, q_max = -1.0, 1.0
    controller = JointCBFQP1DOF(q_min, q_max)
    q, qdot = 0.0, 0.0
    q_ref = 0.9  # reference position inside limits

    for k in range(1000):
        u_safe = controller.solve_qp(q, qdot, q_ref)
        # Forward Euler integration of qdot = u
        qdot = u_safe
        q += dt * qdot
        # Here you would log/plot q, qdot, and u_safe
