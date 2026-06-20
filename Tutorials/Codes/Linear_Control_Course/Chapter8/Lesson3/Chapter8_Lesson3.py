import numpy as np
import control as ct

# Open-loop L(s) = 10 / (s (0.5 s + 1))
s = ct.TransferFunction.s
G = 10 / (s * (0.5 * s + 1.0))
C = 1.0
L = C * G

# Error transfer E(s)/R(s) = 1 / (1 + L(s)) for unity feedback
E_over_R = ct.feedback(1, L)

# Static error constants via low-frequency gain
Kp = ct.dcgain(L)
Kv = ct.dcgain(s * L)       # well-defined here (type 1)
Ka = ct.dcgain(s**2 * L)    # will be zero for this example

print("Kp =", Kp)
print("Kv =", Kv)
print("Ka =", Ka)

def ss_error_step():
    """Numerical steady-state error for unit step input."""
    t, e = ct.step_response(E_over_R)
    return e[-1]

def ss_error_ramp(t_final=50.0):
    """Numerical steady-state error for unit ramp input r(t) = t."""
    t = np.linspace(0.0, t_final, 2000)
    r = t
    t_out, e, _ = ct.forced_response(E_over_R, T=t, U=r)
    return e[-1]

def ss_error_parabolic(t_final=50.0):
    """Numerical steady-state error for unit parabolic input r(t) = 0.5 t^2."""
    t = np.linspace(0.0, t_final, 2000)
    r = 0.5 * t**2
    t_out, e, _ = ct.forced_response(E_over_R, T=t, U=r)
    return e[-1]

print("Step e_ss (simulated):      ", ss_error_step())
print("Ramp e_ss (simulated):      ", ss_error_ramp())
print("Parabolic e_ss (simulated): ", ss_error_parabolic())

# Theoretical values for comparison
e_step_theory = 1.0 / (1.0 + Kp)
e_ramp_theory = 1.0 / Kv
print("Step e_ss (theory):         ", e_step_theory)
print("Ramp e_ss (theory):         ", e_ramp_theory)
