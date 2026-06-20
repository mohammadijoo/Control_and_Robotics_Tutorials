
import numpy as np

# Example: n-DOF joint trajectory, sampled at N steps
# q_d[k], q[k] are arrays in R^n

def joint_space_error(q_d, q, qd_d=None, qd=None, dt=0.001):
    """
    Compute joint-space position and velocity tracking error over a trajectory.

    Parameters
    ----------
    q_d : array_like, shape (N, n)
        Desired joint positions.
    q : array_like, shape (N, n)
        Measured joint positions.
    qd_d : array_like, shape (N, n) or None
        Desired joint velocities (optional).
    qd : array_like, shape (N, n) or None
        Measured joint velocities (optional).
    dt : float
        Sample time.

    Returns
    -------
    e, edot : np.ndarray
        Position and velocity error trajectories.
    ise : float
        Integral of squared position error.
    """
    q_d = np.asarray(q_d)
    q = np.asarray(q)
    assert q_d.shape == q.shape

    e = q_d - q

    if qd_d is not None and qd is not None:
        qd_d = np.asarray(qd_d)
        qd = np.asarray(qd)
        assert qd_d.shape == qd.shape
        edot = qd_d - qd
    else:
        # crude finite-difference approximation
        edot = np.zeros_like(e)
        edot[1:] = (e[1:] - e[:-1]) / dt

    # ISE over all joints and time
    ise = float(np.sum(np.sum(e**2, axis=1)) * dt)
    return e, edot, ise


# Minimal example: 2-DOF arm tracking a sinusoidal reference
if __name__ == "__main__":
    dt = 0.001
    T = 2.0
    t = np.arange(0.0, T, dt)
    N = t.size
    n = 2

    # desired joint trajectories
    qd1 = 0.5 * np.sin(2.0 * np.pi * t)
    qd2 = 0.3 * np.sin(4.0 * np.pi * t)
    q_d = np.stack([qd1, qd2], axis=1)

    # measured: assume a small phase lag and amplitude error
    q = np.stack([
        0.48 * np.sin(2.0 * np.pi * (t - 0.01)),
        0.29 * np.sin(4.0 * np.pi * (t - 0.01)),
    ], axis=1)

    e, edot, ise = joint_space_error(q_d, q, dt=dt)
    print("ISE over horizon T = ", T, " seconds:", ise)
