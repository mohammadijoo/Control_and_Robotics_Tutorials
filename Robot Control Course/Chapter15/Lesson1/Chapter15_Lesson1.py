
import numpy as np
# Nominal robot dynamics (e.g., from roboticstoolbox or your own model)
def M(q):
    return 1.2  # scalar inertia for a simple example

def C(q, qd):
    return 0.0

def g(q):
    return 0.0

# PD gains
Kp = 50.0
Kd = 10.0

# Placeholder for a trained friction model (e.g., sklearn or torch)
class LearnedFrictionModel:
    def __init__(self):
        # In practice, load trained parameters here
        self.w = np.array([0.2, 0.05])  # dummy linear model params

    def predict(self, qd):
        # Features: [qd, sign(qd)]
        phi = np.array([qd, np.sign(qd)])
        return float(self.w @ phi)

fric_model = LearnedFrictionModel()

def joint_controller(q, qd, q_ref, qd_ref, qdd_ref):
    # Nominal computed torque ignoring friction
    q_tilde = q - q_ref
    qd_tilde = qd - qd_ref
    v = qdd_ref - Kd * qd_tilde - Kp * q_tilde
    tau_nom = M(q) * v + C(q, qd) * qd + g(q)

    # Learned friction compensation
    tau_learn = fric_model.predict(qd)

    tau = tau_nom + tau_learn
    return tau

# Example closed-loop simulation step
def step(q, qd, q_ref, qd_ref, qdd_ref, dt=0.001):
    tau = joint_controller(q, qd, q_ref, qd_ref, qdd_ref)
    # True dynamics with unknown friction tau_f(qd) = 0.3*qd + 0.1*sign(qd)
    tau_f_true = 0.3 * qd + 0.1 * np.sign(qd)
    qdd = (tau - tau_f_true) / M(q)
    qd_next = qd + dt * qdd
    q_next = q + dt * qd_next
    return q_next, qd_next
