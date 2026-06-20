import numpy as np

def zeta_from_Mp(Mp_percent: float) -> float:
    """
    Compute damping ratio zeta from percent overshoot Mp_percent (e.g. 10 for 10%).
    Valid for 0 < Mp_percent < 100.
    """
    Mp = Mp_percent / 100.0
    if Mp <= 0.0 or Mp >= 1.0:
        raise ValueError("Mp_percent must be between 0 and 100.")
    lnMp = np.log(Mp)
    zeta = -lnMp / np.sqrt(np.pi**2 + lnMp**2)
    return zeta

def sigma_from_Ts(Ts: float, perc: float = 2.0) -> float:
    """
    Compute approximate real-part bound sigma_max from settling time Ts.
    For perc = 2, use Ts ≈ 4/|sigma|; for perc = 5, Ts ≈ 3/|sigma|.
    """
    if Ts <= 0.0:
        raise ValueError("Ts must be positive.")
    if perc == 2.0:
        return -4.0 / Ts
    elif perc == 5.0:
        return -3.0 / Ts
    else:
        # Generic exponential bound from perc%
        eps = perc / 100.0
        return np.log(eps) / Ts  # negative

def check_pole_constraints(s: complex, Mp_percent: float, Ts: float) -> bool:
    """
    Check if a given pole s satisfies Mp ≤ Mp_percent and Ts ≤ Ts.
    """
    sigma = s.real
    omega_d = abs(s.imag)
    if sigma >= 0.0:
        return False  # unstable or marginal

    # compute zeta from s
    omega_n = np.sqrt(sigma**2 + omega_d**2)
    zeta = -sigma / omega_n

    # required zeta from Mp
    zeta_min = zeta_from_Mp(Mp_percent)
    # required sigma from Ts (2% criterion)
    sigma_max = sigma_from_Ts(Ts, perc=2.0)

    return (zeta >= zeta_min) and (sigma <= sigma_max)

# Example: simple DC motor position loop model for a robotic joint
try:
    import control as ctl  # python-control library
except ImportError:
    ctl = None

if ctl is not None:
    # Plant: G(s) = K_m / (J s^2 + b s)
    J = 0.01   # kg m^2
    b = 0.1    # N m s
    K_m = 0.5  # N m / A (simplified)
    num = [K_m]
    den = [J, b, 0.0]
    G = ctl.tf(num, den)

    # Unity feedback with proportional gain Kp
    Kp = 50.0
    L = Kp * G
    T = ctl.feedback(L, 1)

    poles = ctl.pole(T)
    Mp_spec = 10.0  # 10% overshoot
    Ts_spec = 2.0   # 2 s settling time

    print("Closed-loop poles:", poles)
    for p in poles:
        ok = check_pole_constraints(p, Mp_spec, Ts_spec)
        print(f"Pole {p}: meets specs? {ok}")
else:
    print("python-control not installed; skipping plant example.")
