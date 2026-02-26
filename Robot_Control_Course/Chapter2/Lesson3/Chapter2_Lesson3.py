
import numpy as np

# Example: using pinocchio-style API (pseudo-code)
import pinocchio as pin

# Load robot model from URDF
model = pin.buildModelFromUrdf("my_robot.urdf")
data = model.createData()

def gravity_torque(q):
    """
    Return gravity compensation torque tau_g(q) for configuration q.
    q: numpy array of shape (n,)
    """
    dq = np.zeros(model.nv)
    ddq = np.zeros(model.nv)
    # Inverse dynamics: tau = M(q) ddq + C(q,dq) dq + g(q)
    tau = pin.rnea(model, data, q, dq, ddq)
    return tau

def pd_gravity_control(q, dq, q_d, dq_d, Kp, Kd):
    """
    Joint-space PD + gravity compensation.
    Kp, Kd: diagonal gain matrices (np.ndarray) or scalars.
    """
    e = q_d - q
    de = dq_d - dq
    tau_pd = Kp @ e + Kd @ de
    tau_g = gravity_torque(q)
    return tau_pd + tau_g

# Example usage inside a control loop
n = model.nq
Kp = np.diag([80.0] * n)
Kd = np.diag([15.0] * n)
q_d = np.zeros(n)
dq_d = np.zeros(n)

def control_step(q_meas, dq_meas):
    tau = pd_gravity_control(q_meas, dq_meas, q_d, dq_d, Kp, Kd)
    # send tau to actuators here
    return tau
