import numpy as np

def sat_scalar(u, u_min, u_max):
    """
    Scalar saturation.
    """
    return max(u_min, min(u, u_max))

def sat_vector(u, u_min, u_max):
    """
    Componentwise saturation of vectors (numpy arrays).
    """
    u = np.asarray(u, dtype=float)
    u_min = np.asarray(u_min, dtype=float)
    u_max = np.asarray(u_max, dtype=float)
    return np.minimum(u_max, np.maximum(u_min, u))

def feasible_acc_1d(q, qd, tau_min, tau_max, I, b, g_func):
    """
    Compute the min and max acceleration consistent with torque limits
    for a 1-DOF joint.
    """
    g = g_func(q)
    ddq_min = (tau_min - b * qd - g) / I
    ddq_max = (tau_max - b * qd - g) / I
    return ddq_min, ddq_max

# Example usage:
I = 0.2      # kg m^2
b = 0.05     # N m s/rad
tau_min = -5.0
tau_max = 5.0

def g_func(q):
    # Simple gravity torque model, e.g. for a pendulum-like link
    m = 1.0
    l = 0.5
    g0 = 9.81
    return m * g0 * l * np.sin(q)

q = 0.5       # rad
qd = 0.0      # rad/s

# Commanded torque (from a controller, not modeled here)
tau_cmd = 8.0
tau_act = sat_scalar(tau_cmd, tau_min, tau_max)

ddq_min, ddq_max = feasible_acc_1d(q, qd, tau_min, tau_max, I, b, g_func)

print("Actual torque:", tau_act)
print("Feasible acceleration interval:", ddq_min, ddq_max)
      
