import numpy as np
import control as ctl  # python-control, common in robotics/control
# pip install control

def trailing_zeros(coeffs, tol=1e-9):
    """Count trailing zeros (small coefficients) in polynomial coefficient array.
       Coeffs are ordered from highest power down to constant term."""
    k = 0
    for c in coeffs[::-1]:  # start from constant term
        if abs(c) < tol:
            k += 1
        else:
            break
    return k

def system_type_and_static_constants(G, H=1.0, s_eps=1e-6):
    """
    Compute system type (net number of integrators in L(s) = G(s)H(s))
    and approximate {Kp, Kv, Ka}.
    """
    # Build open-loop L(s)
    if isinstance(H, (int, float)):
        L = G * H
    else:
        L = G * H

    num, den = ctl.tfdata(L)  # returns arrays with shape (1, 1, n)
    num = np.squeeze(num).astype(float)
    den = np.squeeze(den).astype(float)

    n_zeros_at_origin = trailing_zeros(num)
    n_poles_at_origin = trailing_zeros(den)
    sys_type = max(n_poles_at_origin - n_zeros_at_origin, 0)

    # Evaluate static error constants numerically
    # (good enough for engineering use if s_eps is small)
    L0 = ctl.evalfr(L, 0)              # L(0)
    Ls = ctl.evalfr(L, s_eps)          # L(s_eps), s_eps small

    Kp = np.real_if_close(L0)
    Kv = np.real_if_close(s_eps * Ls)
    Ka = np.real_if_close((s_eps ** 2) * Ls)

    return {"type": int(sys_type), "Kp": float(Kp),
            "Kv": float(Kv), "Ka": float(Ka)}

# Example: simple robot joint position loop plant
# Motor + load: G(s) = K / (J s^2 + B s)
J = 0.01   # kg m^2
B = 0.1    # N m s/rad
K = 1.0    # gain (N m / control signal)

s = ctl.TransferFunction.s
G_joint = K / (J * s**2 + B * s)  # position / control signal
L_joint = G_joint                 # unity feedback for simplicity

info = system_type_and_static_constants(L_joint)
print("System type:", info["type"])
print("Kp, Kv, Ka:", info["Kp"], info["Kv"], info["Ka"])

# You can also simulate a step or ramp response to validate predictions:
t, y_step = ctl.step_response(ctl.feedback(L_joint, 1), T=np.linspace(0, 5, 500))
