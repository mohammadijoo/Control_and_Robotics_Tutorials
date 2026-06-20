# Chapter21_Lesson3.py
# Zero Dynamics and Internal System Behavior
#
# Example:
#   G(s) = (s - 1) / ((s + 1)(s + 2)(s + 3))
#
# The realization is
#   x_dot = A x + B u,  y = C x
# with output constraint y = 0.  The zero dynamics are eta_dot = eta,
# hence the internal motion grows although the output is held at zero.

import numpy as np
from scipy import signal
from scipy.integrate import solve_ivp


A = np.array([[0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0],
              [-6.0, -11.0, -6.0]])

B = np.array([[0.0],
              [0.0],
              [1.0]])

C = np.array([[-1.0, 1.0, 0.0]])
D = np.array([[0.0]])


def transfer_zeros_from_ss(A, B, C, D):
    """Return transfer numerator roots for a SISO realization."""
    num, den = signal.ss2tf(A, B, C, D)
    num = np.trim_zeros(num[0], trim="f")
    den = np.trim_zeros(den, trim="f")
    zeros = np.roots(num)
    poles = np.roots(den)
    return zeros, poles, num, den


def output_nulling_feedback(x):
    """
    On the zero-output manifold for this example:
        x2 = x1, x3 = x1.
    Enforcing d/dt(x3 - x1) = 0 gives u = 18*x1.
    """
    return 18.0 * x[0]


def closed_loop_zero_dynamics(t, x):
    u = output_nulling_feedback(x)
    return (A @ x + B.flatten() * u)


def main():
    zeros, poles, num, den = transfer_zeros_from_ss(A, B, C, D)

    print("Numerator coefficients:", num)
    print("Denominator coefficients:", den)
    print("Transmission zero(s):", zeros)
    print("Poles:", poles)

    # Initial state chosen on the output-nulling manifold:
    # y = -x1 + x2 = 0 and y_dot = -x2 + x3 = 0.
    eta0 = 0.02
    x0 = np.array([eta0, eta0, eta0])

    sol = solve_ivp(
        closed_loop_zero_dynamics,
        t_span=(0.0, 5.0),
        y0=x0,
        t_eval=np.linspace(0.0, 5.0, 101),
        rtol=1e-9,
        atol=1e-11,
    )

    y = np.array([(C @ sol.y[:, k] + D.flatten() * output_nulling_feedback(sol.y[:, k]))[0]
                  for k in range(sol.y.shape[1])])

    eta_exact = eta0 * np.exp(sol.t)

    print("\nZero-output simulation")
    print("max |y(t)|:", np.max(np.abs(y)))
    print("eta(5) numerical:", sol.y[0, -1])
    print("eta(5) exact:", eta_exact[-1])
    print("interpretation: output is zero, internal state grows because zero = +1")


if __name__ == "__main__":
    main()
