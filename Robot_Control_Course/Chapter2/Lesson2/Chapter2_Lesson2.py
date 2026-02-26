
import numpy as np

def simulate_joint_pid(
    q0, dq0,
    qd,
    Kp, Kd, Ki,
    J=1.0, b=0.0,
    dt=0.001, T=2.0,
    use_integral=True
):
    """
    Simulate scalar joint dynamics:
        J * ddq + b * dq = tau
    with PD or PID control.
    """
    N = int(T / dt)
    q = np.zeros(N)
    dq = np.zeros(N)
    e_hist = np.zeros(N)
    t = np.linspace(0.0, T, N)

    q[0] = q0
    dq[0] = dq0
    z = 0.0                # integral state
    e_prev = qd - q0
    de_f = 0.0             # filtered derivative
    alpha = 0.2            # derivative filter weight

    for k in range(1, N):
        # For this demo, desired position is constant qd
        e = qd - q[k-1]
        de = (e - e_prev) / dt
        de_f = (1.0 - alpha) * de_f + alpha * de

        if use_integral:
            z += e * dt

        tau = Kp * e + Kd * de_f + (Ki * z if use_integral else 0.0)

        # Joint dynamics: explicit Euler integration
        ddq = (tau - b * dq[k-1]) / J
        dq[k] = dq[k-1] + ddq * dt
        q[k] = q[k-1] + dq[k] * dt

        e_hist[k] = e
        e_prev = e

    return t, q, dq, e_hist

if __name__ == "__main__":
    # Example: J = 0.5, b = 0.05, desired position qd = 1 rad
    J = 0.5
    b = 0.05
    # Choose gains from a second-order specification
    zeta = 0.7
    omega_n = 6.0
    Kp = J * omega_n**2
    Kd = 2.0 * J * zeta * omega_n - b
    Ki = 5.0   # must satisfy Routh-Hurwitz constraint

    t, q, dq, e_hist = simulate_joint_pid(
        q0=0.0, dq0=0.0,
        qd=1.0,
        Kp=Kp, Kd=Kd, Ki=Ki,
        J=J, b=b,
        dt=0.001, T=2.0,
        use_integral=True
    )

    # Plotting (requires matplotlib)
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(t, q, label="q(t)")
    plt.plot(t, np.ones_like(t), "--", label="qd")
    plt.xlabel("time [s]")
    plt.ylabel("position [rad]")
    plt.legend()
    plt.grid(True)
    plt.show()
