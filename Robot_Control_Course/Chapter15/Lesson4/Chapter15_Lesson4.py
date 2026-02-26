
import numpy as np

class ScalarJointWrapper:
    """
    Stability-preserving wrapper for a scalar joint:
        x_dot = -a * x + u
    Nominal controller: u_b = -k * x
    Learned residual: u_l(x) from a regression model
    Wrapper enforces |u_l(x)| <= c * |x|.
    """
    def __init__(self, a: float, k: float, c_scale: float = 0.5, learner=None):
        self.a = a
        self.k = k
        # Choose c = c_scale * (a + k), 0 < c_scale < 1
        self.c = c_scale * (a + k)
        self.learner = learner  # e.g., a NN or GP with .predict(x_array)

    def nominal_control(self, x: float) -> float:
        return -self.k * x

    def learned_residual(self, x: float) -> float:
        if self.learner is None:
            return 0.0
        # learner expects shape (N,1)
        x_arr = np.array([[x]])
        u_l = float(self.learner.predict(x_arr))
        return u_l

    def wrapped_control(self, x: float) -> float:
        u_b = self.nominal_control(x)
        u_l = self.learned_residual(x)
        # Lyapunov-based saturation: |u_l| <= c * |x|
        bound = self.c * abs(x)
        if abs(u_l) > bound:
            u_l = np.sign(u_l) * bound
        return u_b + u_l

# Example usage (with a dummy learner)
class DummyLearner:
    def predict(self, X):
        # Pretend we have learned some residual; here just a cubic nonlinearity
        return 0.8 * X[:, 0]**3

learner = DummyLearner()
wrapper = ScalarJointWrapper(a=1.0, k=4.0, c_scale=0.5, learner=learner)

x = 0.3
u = wrapper.wrapped_control(x)
print("State x =", x, "wrapped control u =", u)
