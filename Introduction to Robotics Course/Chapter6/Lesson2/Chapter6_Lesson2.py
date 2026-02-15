import numpy as np

def eq_ratio(stages):
    """stages: list of (z_motor, z_load) for each gear mesh"""
    N = 1.0
    for z1, z2 in stages:
        N *= (z2 / z1)
    return N

def reflected_inertia(Jm, Jl, N):
    """Return (Req at load side, Req at motor side)."""
    J_at_load = Jl + (N**2) * Jm
    J_at_motor = Jm + Jl / (N**2)
    return J_at_load, J_at_motor

def backlash_torque(theta_m, theta_l, N, b, kt):
    """Piecewise spring backlash model at output."""
    dtheta = theta_m / N - theta_l
    if abs(dtheta) <= b/2:
        return 0.0
    elif dtheta > b/2:
        return kt * (dtheta - b/2)
    else:
        return kt * (dtheta + b/2)

# Example usage
stages = [(20, 80), (18, 54)]  # two stages
N = eq_ratio(stages)
print("N_eq =", N)

Jm, Jl = 2e-4, 5e-3
print("Reflected inertia:", reflected_inertia(Jm, Jl, N))

# static torque for a few relative angles
for d in np.linspace(-0.05, 0.05, 5):
    tm = N*(0.0 + d)  # pick theta_m so that theta_m/N - theta_l = d
    tl = 0.0
    print(d, backlash_torque(tm, tl, N, b=0.02, kt=150.0))
