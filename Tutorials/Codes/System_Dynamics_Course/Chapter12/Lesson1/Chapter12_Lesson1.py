"""
Chapter12_Lesson1.py
System Dynamics (Control Engineering) — Chapter 12, Lesson 1
Sinusoidal Steady-State Response and Frequency Response Definition

Requires: numpy, scipy (scipy.signal), matplotlib (optional for plots)
Optional: control (python-control) for alternative frequency-response utilities.
"""

import numpy as np
from numpy import pi
from scipy import signal

def tf_eval(num, den, s):
    """
    Evaluate a rational transfer function G(s) = N(s)/D(s) at complex s.
    num, den: 1D arrays of polynomial coefficients in descending powers.
    """
    num = np.asarray(num, dtype=np.complex128)
    den = np.asarray(den, dtype=np.complex128)
    return np.polyval(num, s) / np.polyval(den, s)

def fit_sinusoid(t, y, omega):
    """
    Fit y(t) ≈ a*sin(omega*t) + b*cos(omega*t) by least squares.
    Returns amplitude R and phase phi such that y ≈ R*sin(omega*t + phi).
    """
    A = np.column_stack([np.sin(omega * t), np.cos(omega * t)])
    coeff, *_ = np.linalg.lstsq(A, y, rcond=None)
    a, b = coeff
    R = np.sqrt(a*a + b*b)
    phi = np.arctan2(b, a)  # because R*sin(wt+phi)=R*cos(phi)*sin(wt)+R*sin(phi)*cos(wt)
    return R, phi, a, b

def main():
    # Example: standard 2nd-order low-pass
    wn = 5.0        # rad/s
    zeta = 0.2
    num = [wn**2]
    den = [1.0, 2.0*zeta*wn, wn**2]
    G = signal.TransferFunction(num, den)

    # Sinusoidal input
    Um = 1.0
    omega = 4.0  # rad/s
    phi_u = 0.0  # rad

    # Frequency response prediction
    Gjw = tf_eval(num, den, 1j*omega)
    Ym_pred = abs(Gjw) * Um
    phi_y_pred = np.angle(Gjw) + phi_u

    print("G(jw) =", Gjw)
    print("|G(jw)| =", abs(Gjw), "  angle(G(jw)) =", np.angle(Gjw))
    print("Predicted steady-state amplitude Ym =", Ym_pred)
    print("Predicted steady-state phase phi_y (rad) =", phi_y_pred)

    # Time-domain simulation (zero initial conditions)
    t = np.linspace(0.0, 40.0, 40001)  # long horizon to let transients decay
    u = Um * np.sin(omega*t + phi_u)
    tout, y, _ = signal.lsim(G, U=u, T=t)

    # Estimate steady state from final window (e.g., last 10 seconds)
    mask = tout >= (tout[-1] - 10.0)
    R_hat, phi_hat, a_hat, b_hat = fit_sinusoid(tout[mask], y[mask], omega)

    # Normalize phase to be comparable
    def wrap_to_pi(x):
        return (x + np.pi) % (2*np.pi) - np.pi

    print("\nEstimated from simulation (last 10 s):")
    print("Ym_hat =", R_hat)
    print("phi_y_hat (rad) =", wrap_to_pi(phi_hat))

    print("\nErrors:")
    print("Amplitude error:", R_hat - Ym_pred)
    print("Phase error (rad):", wrap_to_pi(phi_hat - phi_y_pred))

    # Optional plotting (uncomment if desired)
    # import matplotlib.pyplot as plt
    # plt.figure()
    # plt.plot(tout, u, label="u(t)")
    # plt.plot(tout, y, label="y(t)")
    # plt.xlim((tout[-1]-5.0, tout[-1]))
    # plt.legend()
    # plt.xlabel("t (s)")
    # plt.grid(True)
    # plt.show()

if __name__ == "__main__":
    main()
