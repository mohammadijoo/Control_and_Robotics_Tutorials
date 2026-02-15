import numpy as np

def bode_asymptotic_real(omega, K, zeros, poles, n_int=0, n_diff=0):
    """
    Approximate Bode magnitude (dB) and phase (deg) for
    G(s) = K * s**n_diff / s**n_int
           * prod(1 + s/z_i) / prod(1 + s/p_k),
    where zeros, poles are lists of real corner frequencies (rad/s).
    """
    omega = np.asarray(omega, dtype=float)
    logw = np.log10(omega)

    # Magnitude in dB: start with gain contribution
    mag_db = 20.0 * np.log10(abs(K)) * np.ones_like(omega)

    # Integrator / differentiator contributions
    # 1/s raises -20 dB/decade, s raises +20 dB/decade
    mag_db += 20.0 * (n_diff - n_int) * logw

    # Phase in degrees
    phase_deg = np.zeros_like(omega)
    if K < 0:
        phase_deg += 180.0
    phase_deg += 90.0 * (n_diff - n_int)

    def phase_piecewise_first_order(omega, wc, sign):
        """
        sign = +1 for zero, -1 for pole.
        Piecewise linear phase approximation:
        0 deg for w <= wc/10,
        +/-90 deg for w >= 10*wc,
        linear in between.
        """
        ph = np.zeros_like(omega)
        w1 = wc / 10.0
        w2 = 10.0 * wc

        # region w <= w1: 0 deg
        # region w >= w2: +/-90 deg
        ph[omega >= w2] = sign * 90.0

        # linear region
        mask = (omega > w1) & (omega < w2)
        # fraction in [0, 1]
        frac = (np.log10(omega[mask]) - np.log10(w1)) / (np.log10(w2) - np.log10(w1))
        ph[mask] = sign * 90.0 * frac
        return ph

    # Real zeros
    for wz in zeros:
        # magnitude: +20 dB/decade beyond wz
        mag_db += 20.0 * np.maximum(0.0, np.log10(omega / wz))
        # phase
        phase_deg += phase_piecewise_first_order(omega, wz, +1.0)

    # Real poles
    for wp in poles:
        # magnitude: -20 dB/decade beyond wp
        mag_db -= 20.0 * np.maximum(0.0, np.log10(omega / wp))
        # phase
        phase_deg += phase_piecewise_first_order(omega, wp, -1.0)

    return mag_db, phase_deg

if __name__ == "__main__":
    # Example: G(s) = 100 (s + 10) / (s (s + 1) (s + 100))
    K = 100.0
    zeros = [10.0]
    poles = [1.0, 100.0]
    n_int = 1    # one integrator
    n_diff = 0

    w = np.logspace(-2, 3, 200)
    mag_db, phase_deg = bode_asymptotic_real(w, K, zeros, poles, n_int, n_diff)

    import matplotlib.pyplot as plt

    plt.figure()
    plt.semilogx(w, mag_db)
    plt.xlabel("omega [rad/s]")
    plt.ylabel("Magnitude [dB]")
    plt.grid(True, which="both")

    plt.figure()
    plt.semilogx(w, phase_deg)
    plt.xlabel("omega [rad/s]")
    plt.ylabel("Phase [deg]")
    plt.grid(True, which="both")

    plt.show()
