import numpy as np

def q_path(s):
    """
    Geometric path q_p(s) for a 2-DOF manipulator.
    s: scalar or numpy array in [0,1].
    Returns: array of shape (..., 2).
    """
    s = np.asarray(s)
    q1 = np.cos(0.5 * np.pi * s)
    q2 = np.sin(0.5 * np.pi * s)
    return np.stack((q1, q2), axis=-1)

def dqds_path(s):
    """
    First path derivative dq_p/ds.
    """
    s = np.asarray(s)
    dq1 = -0.5 * np.pi * np.sin(0.5 * np.pi * s)
    dq2 =  0.5 * np.pi * np.cos(0.5 * np.pi * s)
    return np.stack((dq1, dq2), axis=-1)

def d2qds2_path(s):
    """
    Second path derivative d^2 q_p/ds^2.
    """
    s = np.asarray(s)
    d2q1 = -(0.5 * np.pi)**2 * np.cos(0.5 * np.pi * s)
    d2q2 = -(0.5 * np.pi)**2 * np.sin(0.5 * np.pi * s)
    return np.stack((d2q1, d2q2), axis=-1)

def s_time(t, T):
    """
    Cubic time law: s(t) = 3 (t/T)^2 - 2 (t/T)^3
    with s(0)=0, s(T)=1, s_dot(0)=s_dot(T)=0.
    """
    tau = t / T
    return 3.0 * tau**2 - 2.0 * tau**3

def sdot_time(t, T):
    tau = t / T
    return (6.0 * tau - 6.0 * tau**2) / T

def sddot_time(t, T):
    tau = t / T
    return (6.0 - 12.0 * tau) / (T**2)

def evaluate_trajectory(T=2.0, n_samples=100):
    t = np.linspace(0.0, T, n_samples)
    s = s_time(t, T)
    sdot = sdot_time(t, T)
    sddot = sddot_time(t, T)

    q = q_path(s)
    dqds = dqds_path(s)
    d2qds2 = d2qds2_path(s)

    qdot = dqds * sdot[:, None]
    qddot = d2qds2 * (sdot[:, None]**2) + dqds * sddot[:, None]

    # Check simple joint velocity limits
    qdot_max = np.array([1.0, 1.0])  # rad/s
    if np.any(np.abs(qdot) > qdot_max):
        print("Warning: joint velocity limits exceeded.")
    else:
        print("Joint velocity limits satisfied.")

    return t, q, qdot, qddot

if __name__ == "__main__":
    t, q, qdot, qddot = evaluate_trajectory()
      
