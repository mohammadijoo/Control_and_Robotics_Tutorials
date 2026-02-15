import numpy as np

class ComplexRobotModel:
    def __init__(self, pin_model, pin_data, residual_model=None):
        """
        pin_model, pin_data: nominal rigid-body model (e.g. from pinocchio).
        residual_model: callable r(q, dq, ddq) -> R^n or None.
        """
        self.pin_model = pin_model
        self.pin_data = pin_data
        self.residual_model = residual_model

    def inverse_dynamics_nominal(self, q, dq, ddq):
        """
        Wrapper around a standard recursive Newton-Euler algorithm.
        Any equivalent FK-based implementation from earlier chapters can be used.
        """
        import pinocchio as pin  # or your own implementation
        tau_nom = pin.rnea(self.pin_model, self.pin_data, q, dq, ddq)
        return np.array(tau_nom)

    def residual(self, q, dq, ddq):
        if self.residual_model is None:
            return np.zeros_like(q)
        return np.array(self.residual_model(q, dq, ddq))

    def inverse_dynamics(self, q, dq, ddq):
        """
        tau = tau_nominal - r(q, dq, ddq)
        """
        tau_nom = self.inverse_dynamics_nominal(q, dq, ddq)
        r_val = self.residual(q, dq, ddq)
        return tau_nom - r_val

# Example: linear-in-parameters residual model
class LinearResidual:
    def __init__(self, theta):
        self.theta = np.asarray(theta)

    def __call__(self, q, dq, ddq):
        # Very simple regressor using stacked state
        phi = np.concatenate([q, dq, ddq, np.ones_like(q)])
        # Build block-diagonal regressor for each joint
        n = q.shape[0]
        # Phi_full has shape (n, len(phi))
        Phi_full = np.tile(phi, (n, 1))
        # residual r = Phi_full @ theta, reshaped
        return Phi_full @ self.theta

# Usage (assuming pin_model, pin_data are available)
# residual_model = LinearResidual(theta=np.zeros(3*n + 1))
# robot = ComplexRobotModel(pin_model, pin_data, residual_model)
# tau = robot.inverse_dynamics(q, dq, ddq)
      
