import numpy as np

def availability(mtbf, mttr):
    """Compute steady-state availability A = MTBF / (MTBF + MTTR)."""
    mtbf = float(mtbf)
    mttr = float(mttr)
    return mtbf / (mtbf + mttr)

def cost_rate_age_replacement(lam, C_p, C_f, tau):
    """
    Average cost rate g(tau) for exponential failure rate lam,
    preventive cost C_p, corrective cost C_f, replacement age tau.
    """
    lam = float(lam)
    C_p = float(C_p)
    C_f = float(C_f)
    tau = float(tau)
    if tau <= 0.0:
        raise ValueError("tau must be positive")
    num = C_f + (C_p - C_f) * np.exp(-lam * tau)
    den = 1.0 - np.exp(-lam * tau)
    return lam * num / den

# Example parameters (illustrative only)
mtbf_motor = 2000.0  # hours
mttr_motor = 4.0     # hours
A_motor = availability(mtbf_motor, mttr_motor)
print("Motor availability:", A_motor)

lam = 1.0 / mtbf_motor
C_p = 200.0   # preventive replacement cost
C_f = 800.0   # corrective replacement cost

taus = np.linspace(100.0, 3000.0, 30)
g_vals = [cost_rate_age_replacement(lam, C_p, C_f, tau) for tau in taus]

tau_star = taus[int(np.argmin(g_vals))]
g_star = min(g_vals)
print("Approx optimal replacement age:", tau_star, "hours")
print("Min cost rate:", g_star, "cost units per hour")
      
