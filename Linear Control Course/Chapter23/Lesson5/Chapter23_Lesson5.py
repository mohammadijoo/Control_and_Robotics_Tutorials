import numpy as np
import control as ctl

# Nominal physical and controller parameters
d = 2.0
k = 50.0
Kp = 150.0
Kd = 20.0

def closed_loop_tf(m, omega_h=None):
    """
    Build the closed-loop transfer function for given mass m.
    If omega_h is not None, add an unmodeled high-frequency pole at s = -omega_h.
    """
    # Nominal plant (no unmodeled pole)
    G0 = ctl.tf([1.0], [m, d, k])
    if omega_h is not None:
        # True plant: extra high-frequency pole 1 / (1 + s/omega_h)
        # i.e. multiply by 1 / (1/omega_h * s + 1)
        Gtrue = G0 * ctl.tf([1.0], [1.0 / omega_h, 1.0])
    else:
        Gtrue = G0

    # PD controller
    C = ctl.tf([Kd, Kp], [1.0])

    # Unity feedback closed-loop
    T = ctl.feedback(C * Gtrue, 1.0)
    return T

def second_order_params(m):
    """Return zeta(m) and wn(m) from the derived formulas."""
    wn = np.sqrt((k + Kp) / m)
    zeta = (d + Kd) / (2.0 * np.sqrt(m * (k + Kp)))
    return zeta, wn

m_values = np.linspace(0.5, 2.5, 9)
for m in m_values:
    zeta, wn = second_order_params(m)
    Ts = 8.0 * m / (d + Kd)  # using t_s(m) ≈ 8m/(d+Kd)
    print(f"m = {m:4.2f}, zeta = {zeta:5.3f}, wn = {wn:6.3f}, Ts ≈ {Ts:5.3f} s")

# Example: add an unmodeled high-frequency pole and inspect margins
m_nom = 1.0
omega_h = 30.0  # high-frequency pole
T_true = closed_loop_tf(m_nom, omega_h=omega_h)
L_true = ctl.minreal(ctl.feedback(T_true, -1))  # approximate loop L(s) from T(s)

gm, pm, wg, wp = ctl.margin(L_true)
print("Approximate gain margin:", gm)
print("Approximate phase margin (deg):", pm)
print("Gain crossover frequency (rad/s):", wg)
