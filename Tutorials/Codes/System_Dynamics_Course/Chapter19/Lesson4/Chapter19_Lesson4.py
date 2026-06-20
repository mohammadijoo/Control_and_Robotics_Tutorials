# Chapter19_Lesson4.py
"""System Dynamics — Chapter 19, Lesson 4
Transport delay models and pseudo-distributed approximations.

This script demonstrates:
  (1) Pure delay: y(t) = u(t - T)  <=>  G(s) = exp(-s T)
  (2) Padé rational approximations of exp(-sT)
  (3) Stable 'pseudo-distributed' approximation via an N-lag chain:
          G_N(s) = (1 / (1 + s T/N))^N
  (4) Time-domain simulation of a first-order plant driven by delayed input.

Dependencies: numpy, scipy, matplotlib
"""

import numpy as np
from math import ceil

try:
    from scipy import signal
except Exception as e:
    raise RuntimeError("This script requires SciPy (scipy.signal).") from e

import matplotlib.pyplot as plt


def chain_delay_tf(T: float, N: int):
    """Return (num, den) of (1/(1+sT/N))^N."""
    if T <= 0:
        raise ValueError("T must be positive.")
    if N < 1:
        raise ValueError("N must be >= 1.")
    num = np.array([1.0])
    den = np.array([T / N, 1.0])
    denN = den.copy()
    for _ in range(N - 1):
        denN = np.convolve(denN, den)
    return num, denN


def pade_delay_tf(T: float, n: int):
    """Return (num, den) for n-th order Padé approximation of exp(-sT)."""
    if T <= 0:
        raise ValueError("T must be positive.")
    if n < 1:
        raise ValueError("n must be >= 1.")
    # SciPy provides a standard Padé routine for delays.
    num, den = signal.pade(T, n)
    # SciPy returns lists; cast to numpy arrays
    return np.array(num, dtype=float), np.array(den, dtype=float)


def series_tf(num1, den1, num2, den2):
    """Multiply two transfer functions."""
    return np.convolve(num1, num2), np.convolve(den1, den2)


def step_response(num, den, t):
    sys = signal.TransferFunction(num, den)
    tout, y = signal.step(sys, T=t)
    return tout, y


def freq_response(num, den, w):
    sys = signal.TransferFunction(num, den)
    w, H = signal.freqresp(sys, w=w)
    return w, H


def simulate_true_delay_first_order(T=1.0, a=1.0, b=1.0, dt=1e-3, t_end=12.0):
    """Simulate ydot = -a y + b u(t-T) with u(t)=1 (unit step) using a buffer."""
    if dt <= 0:
        raise ValueError("dt must be positive.")
    n_steps = int(ceil(t_end / dt)) + 1
    delay_steps = int(round(T / dt))
    # Buffer stores past u values; we use a unit step u(t)=1, but keep generic.
    buf = np.zeros(delay_steps + 1)
    y = 0.0
    ys = np.zeros(n_steps)
    ts = np.arange(n_steps) * dt

    for k in range(n_steps):
        u = 1.0  # step input
        # read delayed value
        u_del = buf[0]
        # shift buffer left, append current u
        buf[:-1] = buf[1:]
        buf[-1] = u

        # Euler update
        y = y + dt * (-a * y + b * u_del)
        ys[k] = y

    return ts, ys


def main():
    # Delay parameters
    T = 1.0
    n_pade = 2
    N_chain = 8

    # Plant: Gp(s) = 1/(s+1)
    num_p = np.array([1.0])
    den_p = np.array([1.0, 1.0])

    # Delay approximations
    num_pade, den_pade = pade_delay_tf(T, n_pade)
    num_chain, den_chain = chain_delay_tf(T, N_chain)

    # Combined plant+delay approximations
    num_pd, den_pd = series_tf(num_p, den_p, num_pade, den_pade)
    num_cd, den_cd = series_tf(num_p, den_p, num_chain, den_chain)

    # Step responses (rational approximations)
    t = np.linspace(0, 12, 2000)
    t1, y_pade = step_response(num_pd, den_pd, t)
    t2, y_chain = step_response(num_cd, den_cd, t)

    # 'True delay' simulation by buffer
    t3, y_true = simulate_true_delay_first_order(T=T, a=1.0, b=1.0, dt=2e-3, t_end=12.0)

    plt.figure()
    plt.plot(t3, y_true, label="True delay (buffer sim)")
    plt.plot(t1, y_pade, label=f"Padé [{n_pade}/{n_pade}]")
    plt.plot(t2, y_chain, label=f"Chain N={N_chain}")
    plt.xlabel("t [s]")
    plt.ylabel("y(t)")
    plt.title("First-order plant with transport delay: step response")
    plt.grid(True)
    plt.legend()

    # Frequency-domain comparison of delay only
    w = np.logspace(-2, 2, 800)
    _, H_pade = freq_response(num_pade, den_pade, w)
    _, H_chain = freq_response(num_chain, den_chain, w)
    H_ideal = np.exp(-1j * w * T)

    plt.figure()
    plt.semilogx(w, np.unwrap(np.angle(H_ideal)), label="Ideal delay phase")
    plt.semilogx(w, np.unwrap(np.angle(H_pade)), label=f"Padé [{n_pade}/{n_pade}] phase")
    plt.semilogx(w, np.unwrap(np.angle(H_chain)), label=f"Chain N={N_chain} phase")
    plt.xlabel("ω [rad/s]")
    plt.ylabel("Phase [rad]")
    plt.title("Delay phase: ideal vs approximations")
    plt.grid(True)
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()
