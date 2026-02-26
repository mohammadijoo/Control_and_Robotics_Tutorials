
import math

def damping_ratio_from_overshoot(Mp_star):
    """
    Compute zeta from allowed peak overshoot Mp_star (0 < Mp_star < 1).
    """
    if Mp_star <= 0.0 or Mp_star >= 1.0:
        raise ValueError("Mp_star must be in (0, 1).")
    logMp = math.log(Mp_star)
    return -logMp / math.sqrt(math.pi**2 + logMp**2)

def pd_gains_from_specs(J, B, Mp_star, Ts_star):
    """
    Compute (Kp, Kd) for a single joint with inertia J and viscous friction B,
    given Mp_star (max overshoot) and Ts_star (desired settling time).
    """
    zeta = damping_ratio_from_overshoot(Mp_star)
    wn = 4.0 / (zeta * Ts_star)
    Kp = J * wn**2
    Kd = 2.0 * J * zeta * wn - B
    return Kp, max(Kd, 0.0)  # enforce nonnegative derivative gain

def pd_control_update(q, qd, dq, dt, Kp, Kd):
    """
    Discrete-time PD controller (no integral term).
    q  : current joint position
    qd : desired joint position
    dq : current joint velocity
    dt : time step (unused here, but useful if integrating additional dynamics)
    """
    e = q - qd
    de = dq  # assuming qd is constant
    tau = -Kp * e - Kd * de
    return tau

if __name__ == "__main__":
    J = 0.5    # kg m^2
    B = 0.05   # N m s/rad
    Mp_star = 0.1   # 10% overshoot
    Ts_star = 0.5   # 0.5 s settling time

    Kp, Kd = pd_gains_from_specs(J, B, Mp_star, Ts_star)
    print(f"Kp = {Kp:.3f}, Kd = {Kd:.3f}")

    # Example control step
    q = 0.1      # rad
    qd = 0.0     # rad
    dq = 0.0     # rad/s
    dt = 0.001   # s
    tau = pd_control_update(q, qd, dq, dt, Kp, Kd)
    print(f"Control torque tau = {tau:.3f} N m")
