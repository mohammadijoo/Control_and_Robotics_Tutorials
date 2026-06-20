# Chapter18_Lesson2.py
# Lagrangian modeling of a cart-pendulum system using generalized coordinates q=[x, theta]
# Includes (1) symbolic derivation check with SymPy, and (2) numerical simulation with SciPy.

import numpy as np
from math import sin, cos
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

try:
    import sympy as sp
    SYMPY_AVAILABLE = True
except Exception:
    SYMPY_AVAILABLE = False


def symbolic_derivation():
    if not SYMPY_AVAILABLE:
        print("SymPy not available; skipping symbolic derivation.")
        return

    t = sp.symbols('t', real=True)
    M, m, l, J, g, k, c, b, u = sp.symbols('M m l J g k c b u', positive=True, real=True)
    x = sp.Function('x')(t)
    th = sp.Function('th')(t)
    xd = sp.diff(x, t)
    thd = sp.diff(th, t)

    T = sp.Rational(1, 2) * (M + m) * xd**2 + m * l * sp.cos(th) * xd * thd + sp.Rational(1, 2) * (J + m * l**2) * thd**2
    V = sp.Rational(1, 2) * k * x**2 + m * g * l * (1 - sp.cos(th))
    D = sp.Rational(1, 2) * c * xd**2 + sp.Rational(1, 2) * b * thd**2
    L = T - V

    q = [x, th]
    qd = [xd, thd]
    Q = [u, 0]

    eom = []
    for qi, qdi, Qi in zip(q, qd, Q):
        expr = sp.diff(sp.diff(L, qdi), t) - sp.diff(L, qi) + sp.diff(D, qdi) - Qi
        eom.append(sp.simplify(sp.expand(expr)))

    print("\nSymbolic Euler-Lagrange equations (left-hand sides = 0):")
    for i, eq in enumerate(eom, start=1):
        print(f"Eq{i}:", sp.factor(eq))


def cart_pendulum_rhs(t, state, p):
    x, th, xd, thd = state
    M = p["M"]
    m = p["m"]
    l = p["l"]
    J = p["J"]
    g = p["g"]
    k = p["k"]
    c = p["c"]
    b = p["b"]

    # Example actuation: smooth pulse on the cart
    u = 2.0 if 0.5 <= t <= 2.5 else 0.0

    # M(q) qdd = rhs
    M11 = M + m
    M12 = m * l * cos(th)
    M21 = M12
    M22 = J + m * l * l

    rhs1 = u - c * xd - k * x + m * l * sin(th) * thd * thd
    rhs2 = -b * thd - m * g * l * sin(th)

    det = M11 * M22 - M12 * M21
    xdd = (rhs1 * M22 - rhs2 * M12) / det
    thdd = (M11 * rhs2 - M21 * rhs1) / det
    return np.array([xd, thd, xdd, thdd], dtype=float)


def total_energy(state, p):
    x, th, xd, thd = state
    M = p["M"]
    m = p["m"]
    l = p["l"]
    J = p["J"]
    g = p["g"]
    k = p["k"]

    T = 0.5 * (M + m) * xd * xd + m * l * np.cos(th) * xd * thd + 0.5 * (J + m * l * l) * thd * thd
    V = 0.5 * k * x * x + m * g * l * (1.0 - np.cos(th))
    return T + V


def simulate():
    p = dict(M=1.0, m=0.25, l=0.5, J=0.02, g=9.81, k=8.0, c=0.35, b=0.05)
    x0 = np.array([0.05, 0.35, 0.0, 0.0], dtype=float)

    sol = solve_ivp(
        fun=lambda t, y: cart_pendulum_rhs(t, y, p),
        t_span=(0.0, 10.0),
        y0=x0,
        method="RK45",
        max_step=0.01,
        rtol=1e-8,
        atol=1e-9,
    )

    E = np.array([total_energy(sol.y[:, i], p) for i in range(sol.y.shape[1])])

    fig1 = plt.figure(figsize=(8, 4))
    plt.plot(sol.t, sol.y[0], label="x (m)")
    plt.plot(sol.t, sol.y[1], label="theta (rad)")
    plt.xlabel("Time (s)")
    plt.ylabel("States")
    plt.title("Cart-Pendulum Generalized Coordinates")
    plt.grid(True)
    plt.legend()
    fig1.tight_layout()
    fig1.savefig("Chapter18_Lesson2_states.png", dpi=150)

    fig2 = plt.figure(figsize=(8, 4))
    plt.plot(sol.t, E)
    plt.xlabel("Time (s)")
    plt.ylabel("Energy (J)")
    plt.title("Total Mechanical Energy (with damping and input pulse)")
    plt.grid(True)
    fig2.tight_layout()
    fig2.savefig("Chapter18_Lesson2_energy.png", dpi=150)

    out = np.column_stack([sol.t, sol.y.T, E])
    np.savetxt(
        "Chapter18_Lesson2_simulation.csv",
        out,
        delimiter=",",
        header="t,x,theta,xdot,thetadot,energy",
        comments=""
    )
    print("Saved: Chapter18_Lesson2_states.png, Chapter18_Lesson2_energy.png, Chapter18_Lesson2_simulation.csv")


if __name__ == "__main__":
    symbolic_derivation()
    simulate()
