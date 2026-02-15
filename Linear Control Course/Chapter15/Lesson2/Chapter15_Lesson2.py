import numpy as np
import matplotlib.pyplot as plt

# Optional: python-control, widely used in robotics and control courses
try:
    import control  # python-control library
except ImportError:
    control = None

def loop_transfer_function(s, K=5.0):
    """
    Example loop L(s) = K / ((s + 0.1)(s + 1)),
    representing a lightly damped servo-like plant with proportional gain K.
    """
    return K / ((s + 0.1) * (s + 1.0))

def nyquist_data(K=5.0, w_min=1e-2, w_max=1e2, n_points=4000):
    # Positive frequencies
    w = np.logspace(np.log10(w_min), np.log10(w_max), n_points)
    s = 1j * w
    L = loop_transfer_function(s, K=K)
    return w, L

def count_encirclements(L):
    """
    Approximate encirclements of -1 by monitoring the winding of 1 + L(jw)
    around the origin. We compute the unwrapped phase and divide by 2*pi.
    Positive result corresponds to counterclockwise; we negate to match
    the clockwise-encirclement convention N = P - Z (for P = 0 here).
    """
    F = 1.0 + L
    phi = np.unwrap(np.angle(F))
    dphi = np.diff(phi)
    total_phase_change = np.sum(dphi)
    N_ccw = total_phase_change / (2.0 * np.pi)
    N_clockwise = -N_ccw
    return N_clockwise

def main():
    K = 5.0
    w, L = nyquist_data(K=K)

    # Nyquist plot (positive frequencies only; mirror is conjugate)
    plt.figure()
    plt.plot(L.real, L.imag, label="L(jw), w > 0")
    plt.plot(L.real, -L.imag, "--", label="L(jw), w < 0 (mirror)")
    plt.scatter([-1.0], [0.0], marker="x", s=80, label="-1 point")
    plt.axhline(0, linewidth=0.5)
    plt.axvline(0, linewidth=0.5)
    plt.xlabel("Re{L(jw)}")
    plt.ylabel("Im{L(jw)}")
    plt.title(f"Nyquist plot, K = {K}")
    plt.legend()
    plt.grid(True)

    # Encirclement count
    N = count_encirclements(L)
    print(f"Approximate clockwise encirclements of -1: N = {N:.2f}")

    plt.show()

if __name__ == "__main__":
    main()
