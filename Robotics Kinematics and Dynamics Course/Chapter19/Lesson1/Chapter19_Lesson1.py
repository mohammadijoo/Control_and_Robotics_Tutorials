import numpy as np

def regressor(q, qd, qdd):
    """
    Return Y(q, qd, qdd) for the 1-DOF rotary link.
    q, qd, qdd: scalars or NumPy arrays of the same shape.
    """
    q = np.array(q)
    qd = np.array(qd)
    qdd = np.array(qdd)
    sgn = np.sign(qd)
    return np.vstack((qdd, qd, sgn, np.sin(q))).T  # shape (N, 4)

# "True" parameters: [I, b, fc, mgL]
theta_true = np.array([0.8, 0.05, 0.2, 3.0])

# Nominal parameters (e.g., CAD)
theta_nominal = np.array([1.0, 0.02, 0.1, 2.5])

# Sample trajectory
t = np.linspace(0.0, 2.0, 201)
q = 0.5 * np.sin(2.0 * np.pi * t)
qd = np.gradient(q, t)
qdd = np.gradient(qd, t)

Y = regressor(q, qd, qdd)

tau_true = Y @ theta_true
tau_nominal = Y @ theta_nominal

rmse = np.sqrt(np.mean((tau_true - tau_nominal)**2))
print("Torque RMSE between true and nominal model:", rmse)
      
