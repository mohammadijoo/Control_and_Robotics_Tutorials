import numpy as np

def finite_difference_jerk(q_samples, dt):
    """
    Compute discrete joint jerk from sampled joint positions.
    Parameters
    ----------
    q_samples : ndarray, shape (N+1, n_joints)
        Sampled joint positions at times t_k = k*dt, k = 0,...,N.
    dt : float
        Sampling period.

    Returns
    -------
    jerk : ndarray, shape (N-2, n_joints)
        Approximate jerk at times t_1,...,t_{N-2}.
    """
    q_samples = np.asarray(q_samples)
    if q_samples.ndim == 1:
        q_samples = q_samples[:, None]

    N_plus_1, n_joints = q_samples.shape
    if N_plus_1 < 4:
        raise ValueError("Need at least 4 samples to compute third derivative")

    N = N_plus_1 - 1
    jerk = np.zeros((N - 2, n_joints))
    dt3 = dt ** 3

    for k in range(1, N - 1):
        jerk[k - 1, :] = (
            q_samples[k + 2, :]
            - 3.0 * q_samples[k + 1, :]
            + 3.0 * q_samples[k, :]
            - q_samples[k - 1, :]
        ) / dt3

    return jerk


def jerk_smoothness_index(q_samples, dt, W=None):
    """
    Compute integrated squared jerk index via discrete approximation.
    J = dt * sum_k j_k^T W j_k.
    """
    q_samples = np.asarray(q_samples)
    jerk = finite_difference_jerk(q_samples, dt)

    n_joints = jerk.shape[1]
    if W is None:
        W = np.eye(n_joints)
    W = np.asarray(W)
    if W.shape != (n_joints, n_joints):
        raise ValueError("Weight matrix W must have shape (n_joints, n_joints)")

    # compute sum over time
    J = 0.0
    for j_k in jerk:
        J += float(j_k.T @ W @ j_k)
    J *= dt
    return J


if __name__ == "__main__":
    # Example: quintic interpolation between two joint configurations
    # q(t) = a0 + a1 t + ... + a5 t^5 chosen to satisfy boundary conditions.
    # Here we simply construct a smooth scalar trajectory using numpy.
    T = 2.0
    dt = 0.01
    t = np.arange(0.0, T + dt, dt)
    N_plus_1 = t.size

    # Single joint example: smooth "S"-shaped motion from 0 to 1 rad
    s = t / T
    q = 10 * s**3 - 15 * s**4 + 6 * s**5  # C^2 quintic polynomial
    q_samples = q[:, None]  # shape (N+1, 1)

    j = finite_difference_jerk(q_samples, dt)
    J_index = jerk_smoothness_index(q_samples, dt)

    print("Discrete jerk shape:", j.shape)
    print("Jerk-based smoothness index J =", J_index)
      
