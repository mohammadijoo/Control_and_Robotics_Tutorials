import numpy as np

# Discrete-time grid for testing
t = np.linspace(0.0, 5.0, 501)
dt = t[1] - t[0]

# Define some test inputs
def u1(t):
    # Unit step
    return (t >= 0.0).astype(float)

def u2(t):
    # Ramp
    return t

def u_shifted(t, tau):
    return u1(t - tau)

# System definitions (vectorized in t)
def S1(u):
    k = 2.0
    return k * u

def S2(u, t):
    return t * u

def S3(u):
    return u**2

def check_linearity(S, t, use_t=False):
    alpha, beta = 1.5, -0.7
    u1_vec = u1(t)
    u2_vec = u2(t)
    u_combo = alpha * u1_vec + beta * u2_vec

    if use_t:
        y_combo = S(u_combo, t)
        y_lin = alpha * S(u1_vec, t) + beta * S(u2_vec, t)
    else:
        y_combo = S(u_combo)
        y_lin = alpha * S(u1_vec) + beta * S(u2_vec)

    err = np.linalg.norm(y_combo - y_lin, ord=2)
    return err

def check_time_invariance(S, t, tau=0.5, use_t=False):
    u_orig = u1(t)
    u_tau = u_shifted(t, tau)

    if use_t:
        y_from_shifted = S(u_tau, t)
        y_orig = S(u_orig, t)
    else:
        y_from_shifted = S(u_tau)
        y_orig = S(u_orig)

    # Shift the original output numerically by tau
    y_orig_shifted = np.interp(t, t + tau, y_orig, left=0.0, right=y_orig[-1])

    err = np.linalg.norm(y_from_shifted - y_orig_shifted, ord=2)
    return err

print("S1 linearity error:", check_linearity(S1, t))
print("S1 time invariance error:", check_time_invariance(S1, t))

print("S2 linearity error:", check_linearity(lambda u: S2(u, t), t, use_t=False))
print("S2 time invariance error:", check_time_invariance(lambda u: S2(u, t), t))

print("S3 linearity error:", check_linearity(S3, t))
print("S3 time invariance error:", check_time_invariance(S3, t))
