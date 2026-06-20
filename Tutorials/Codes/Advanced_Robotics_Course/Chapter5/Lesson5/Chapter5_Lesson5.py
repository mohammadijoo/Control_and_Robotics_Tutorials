import numpy as np

def topp_1d_path(s_f, N, v_max, a_max):
    """
    Time-parameterization for 1D path q(s) = s on [0, s_f] with
    |sdot| <= v_max, |sddot| <= a_max.
    Returns arrays s, sdot, t.
    """
    # Discretize path
    s = np.linspace(0.0, s_f, N + 1)
    ds = np.diff(s)

    # Precompute velocity limit curve (here constant)
    sdot_max = np.full_like(s, v_max, dtype=float)

    # Forward pass
    sdot_fwd = np.zeros_like(s)
    for k in range(N):
        # Max accel
        sdot_cand_sq = sdot_fwd[k]**2 + 2.0 * a_max * ds[k]
        sdot_cand = np.sqrt(max(0.0, sdot_cand_sq))
        sdot_fwd[k + 1] = min(sdot_cand, sdot_max[k + 1])

    # Backward pass
    sdot = sdot_fwd.copy()
    for k in reversed(range(N)):
        # Max decel (note: a_min = -a_max)
        sdot_cand_sq = sdot[k + 1]**2 + 2.0 * a_max * ds[k]
        sdot_cand = np.sqrt(max(0.0, sdot_cand_sq))
        sdot[k] = min(sdot[k], sdot_cand, sdot_max[k])

    # Time integration using trapezoidal rule
    t = np.zeros_like(s)
    for k in range(N):
        if sdot[k] + sdot[k + 1] <= 1e-9:
            raise RuntimeError("Profile infeasible: zero velocity segment")
        t[k + 1] = t[k] + 2.0 * ds[k] / (sdot[k] + sdot[k + 1])

    return s, sdot, t

if __name__ == "__main__":
    s_f = 1.0      # path length
    N = 100        # number of segments
    v_max = 1.0    # max speed
    a_max = 2.0    # max accel

    s, sdot, t = topp_1d_path(s_f, N, v_max, a_max)
    print("Final time T =", t[-1])
      
