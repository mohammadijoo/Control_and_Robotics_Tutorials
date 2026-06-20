
import numpy as np

# Example: n-DOF joint-space PD with torque saturation
class SaturatedPDController:
    def __init__(self, Kp, Kd, tau_min, tau_max):
        """
        Kp, Kd: diagonal gains as 1D numpy arrays of length n
        tau_min, tau_max: joint-wise torque limits (1D arrays)
        """
        self.Kp = np.asarray(Kp)
        self.Kd = np.asarray(Kd)
        self.tau_min = np.asarray(tau_min)
        self.tau_max = np.asarray(tau_max)

    @staticmethod
    def saturate(tau_nom, tau_min, tau_max):
        # Component-wise projection onto [tau_min, tau_max]
        return np.minimum(np.maximum(tau_nom, tau_min), tau_max)

    def compute_tau(self, q, qd, q_ref, qd_ref):
        e = q - q_ref
        ed = qd - qd_ref
        tau_nom = -self.Kp * e - self.Kd * ed
        tau = self.saturate(tau_nom, self.tau_min, self.tau_max)
        return tau

# Example usage in a simulation loop with Pinocchio for dynamics
import pinocchio as pin

robot = ...  # load robot model
controller = SaturatedPDController(
    Kp=np.array([100.0, 80.0, 60.0]),
    Kd=np.array([20.0, 16.0, 12.0]),
    tau_min=np.array([-50.0, -40.0, -30.0]),
    tau_max=np.array([50.0, 40.0, 30.0])
)

dt = 0.001
q = robot.q0.copy()
qd = np.zeros_like(q)
q_ref = q.copy()
qd_ref = np.zeros_like(q)

for k in range(10000):
    # Compute dynamics terms
    pin.computeAllTerms(robot.model, robot.data, q, qd)
    M = robot.data.M
    b = robot.data.nle  # C(q,qd)*qd + g(q)

    tau = controller.compute_tau(q, qd, q_ref, qd_ref)

    # Forward dynamics: qdd = M^{-1}(tau - b)
    qdd = np.linalg.solve(M, tau - b)

    # Simple Euler integration
    qd = qd + dt * qdd
    q = q + dt * qd
