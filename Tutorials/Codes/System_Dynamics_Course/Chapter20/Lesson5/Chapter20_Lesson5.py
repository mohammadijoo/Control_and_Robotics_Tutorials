# Chapter20_Lesson5.py
"""
System Dynamics — Chapter 20, Lesson 5
Integrated Case Studies and Projects:
(Mechatronic System, Vehicle Dynamics, Thermal–Fluid System)

This script provides:
1) Mechatronic DC motor + load with saturation + nonlinear friction (smooth Coulomb/Stribeck).
2) Vehicle planar bicycle model with nonlinear tire (tanh saturation) under periodic steering.
3) Thermal–fluid CSTR (mass + energy balance) with periodic inlet-temperature forcing.

For each case, we:
- define ODE f(t, x)
- simulate with solve_ivp
- compute a simple Poincaré map for periodically forced systems
- estimate a crude largest Lyapunov exponent via two-trajectory renormalization

Dependencies: numpy, scipy
"""
from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from typing import Callable, Tuple
from scipy.integrate import solve_ivp

# -----------------------------
# Utilities
# -----------------------------
def rk_event_sample(ts: np.ndarray, xs: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Simple linear interpolation of state trajectory on a target time grid."""
    out = np.empty((len(t_grid), xs.shape[0]))
    for i, tg in enumerate(t_grid):
        j = np.searchsorted(ts, tg) - 1
        j = np.clip(j, 0, len(ts) - 2)
        t0, t1 = ts[j], ts[j + 1]
        w = 0.0 if t1 == t0 else (tg - t0) / (t1 - t0)
        out[i] = (1 - w) * xs[:, j] + w * xs[:, j + 1]
    return out

def poincare_section(f: Callable[[float, np.ndarray], np.ndarray],
                     x0: np.ndarray,
                     T: float,
                     n_transient: int,
                     n_points: int,
                     t_per_period: int = 200) -> np.ndarray:
    """
    Sample the Poincaré map x(kT) for a T-periodic system xdot=f(t,x), f(t+T,x)=f(t,x).
    We integrate for (n_transient+n_points) periods and return the last n_points samples.
    """
    t0 = 0.0
    x = x0.copy()
    samples = []
    for k in range(n_transient + n_points):
        sol = solve_ivp(f, (t0, t0 + T), x, max_step=T / t_per_period, rtol=1e-8, atol=1e-10)
        x = sol.y[:, -1]
        t0 += T
        if k >= n_transient:
            samples.append(x.copy())
    return np.array(samples)

def lyapunov_largest(f: Callable[[float, np.ndarray], np.ndarray],
                     x0: np.ndarray,
                     T: float,
                     n_steps: int,
                     dt: float,
                     delta0: float = 1e-8) -> float:
    """
    Crude largest Lyapunov exponent estimate using two nearby trajectories with renormalization.
    Returns lambda ~ (1/(n_steps*dt)) * sum log(||d||/delta0).
    """
    rng = np.random.default_rng(0)
    d0 = rng.normal(size=x0.shape)
    d0 = delta0 * d0 / np.linalg.norm(d0)

    x = x0.copy()
    y = x0.copy() + d0
    s = 0.0
    t = 0.0

    def step(z, t, dt):
        # Fixed-step RK4
        k1 = f(t, z)
        k2 = f(t + 0.5*dt, z + 0.5*dt*k1)
        k3 = f(t + 0.5*dt, z + 0.5*dt*k2)
        k4 = f(t + dt, z + dt*k3)
        return z + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

    for _ in range(n_steps):
        x = step(x, t, dt)
        y = step(y, t, dt)
        d = y - x
        nd = np.linalg.norm(d)
        if nd == 0:
            continue
        s += np.log(nd / delta0)
        d = (delta0 / nd) * d
        y = x + d
        t += dt
        # keep time bounded for periodic forcing evaluation
        if T is not None and T > 0:
            t = t % T
    return s / (n_steps * dt)

# -----------------------------
# Case Study 1: Mechatronic DC motor + load
# -----------------------------
@dataclass
class MotorParams:
    J: float = 2e-3      # kg m^2
    b: float = 1e-3      # viscous friction N m s
    Kt: float = 0.05     # torque constant N m / A
    Ke: float = 0.05     # back-emf V / (rad/s)
    R: float = 1.0       # Ohm
    L: float = 5e-3      # H
    tau_c: float = 0.02  # Coulomb friction N m
    tau_s: float = 0.03  # static friction amplitude N m
    w_s: float = 2.0     # Stribeck speed rad/s
    u_max: float = 12.0  # V (saturation)

def sat(u, umax):
    return np.clip(u, -umax, umax)

def stribeck_friction(w, p: MotorParams):
    # Smooth sign approx via tanh
    sign = np.tanh(50.0 * w)
    tau = (p.tau_c + (p.tau_s - p.tau_c) * np.exp(-(np.abs(w)/p.w_s)**2)) * sign
    return tau

def motor_ode(p: MotorParams, u_fun: Callable[[float], float]) -> Callable[[float, np.ndarray], np.ndarray]:
    # state x=[theta, w, i]
    def f(t, x):
        th, w, i = x
        u = sat(u_fun(t), p.u_max)
        tau_f = p.b * w + stribeck_friction(w, p)
        dth = w
        dw = (p.Kt * i - tau_f) / p.J
        di = (u - p.R * i - p.Ke * w) / p.L
        return np.array([dth, dw, di], dtype=float)
    return f

# -----------------------------
# Case Study 2: Vehicle bicycle model (planar)
# -----------------------------
@dataclass
class VehicleParams:
    m: float = 1500.0
    Iz: float = 2500.0
    a: float = 1.2   # CG to front axle
    b: float = 1.6   # CG to rear axle
    Ux: float = 20.0 # constant longitudinal speed m/s
    Cf: float = 80000.0 # N/rad (linear cornering stiffness)
    Cr: float = 90000.0
    alpha_sat: float = 0.15 # rad slip saturation scale
    delta0: float = 0.06    # rad steering amplitude
    T: float = 1.0          # forcing period (s)

def tire_force(alpha, C, alpha_sat):
    # Smooth saturation: F = C * alpha_sat * tanh(alpha/alpha_sat)
    return C * alpha_sat * np.tanh(alpha / alpha_sat)

def bicycle_ode(p: VehicleParams) -> Callable[[float, np.ndarray], np.ndarray]:
    # states x=[vy, r] lateral velocity and yaw rate
    def delta(t):
        return p.delta0 * np.sin(2*np.pi*t/p.T)
    def f(t, x):
        vy, r = x
        Ux = p.Ux
        # slip angles (small-angle kinematics)
        alpha_f = (vy + p.a*r)/Ux - delta(t)
        alpha_r = (vy - p.b*r)/Ux
        Fyf = -tire_force(alpha_f, p.Cf, p.alpha_sat)
        Fyr = -tire_force(alpha_r, p.Cr, p.alpha_sat)
        dvy = (Fyf + Fyr)/p.m - Ux*r
        dr  = (p.a*Fyf - p.b*Fyr)/p.Iz
        return np.array([dvy, dr], dtype=float)
    return f

# -----------------------------
# Case Study 3: Thermal–fluid CSTR (mass+energy)
# -----------------------------
@dataclass
class CSTRParams:
    V: float = 1.0        # m^3
    rho: float = 1000.0   # kg/m^3
    Cp: float = 4180.0    # J/(kg K)
    q: float = 1e-3       # m^3/s (flow rate)
    CAf: float = 1.0      # kmol/m^3
    Tf0: float = 300.0    # K mean inlet temperature
    dTf: float = 5.0      # K forcing amplitude
    T: float = 2.0        # s forcing period
    k0: float = 7.2e10    # 1/s
    E: float = 8.314e4    # J/mol
    Rg: float = 8.314     # J/(mol K)
    dH: float = -5e7      # J/kmol (exothermic negative)
    UA: float = 5e4       # J/(s K)
    Tc: float = 295.0     # K coolant temp

def cstr_ode(p: CSTRParams) -> Callable[[float, np.ndarray], np.ndarray]:
    # states x=[CA, T]
    def Tf(t):
        return p.Tf0 + p.dTf*np.sin(2*np.pi*t/p.T)
    def rate(CA, T):
        # Arrhenius: k(T)=k0 exp(-E/(Rg T))
        k = p.k0*np.exp(-p.E/(p.Rg*T))
        return k*CA
    def f(t, x):
        CA, T = x
        rA = rate(CA, T)
        dCA = (p.q/p.V)*(p.CAf - CA) - rA
        # energy balance
        dT = (p.q/p.V)*(Tf(t) - T) \
             + (-p.dH/(p.rho*p.Cp))*rA \
             - (p.UA/(p.rho*p.Cp*p.V))*(T - p.Tc)
        return np.array([dCA, dT], dtype=float)
    return f

# -----------------------------
# Main demo (prints key outputs)
# -----------------------------
def main():
    # 1) Motor
    mp = MotorParams()
    u_fun = lambda t: 8.0*np.sin(2*np.pi*0.8*t)  # periodic voltage
    f1 = motor_ode(mp, u_fun)
    x0_1 = np.array([0.0, 0.0, 0.0])
    T1 = 1.25  # forcing period
    P1 = poincare_section(f1, x0_1, T1, n_transient=50, n_points=20)
    lam1 = lyapunov_largest(f1, P1[-1], T1, n_steps=8000, dt=1e-3, delta0=1e-8)
    print("[Motor] Poincare last sample:", P1[-1])
    print("[Motor] crude largest Lyapunov ~", lam1, "1/s")

    # 2) Vehicle
    vp = VehicleParams()
    f2 = bicycle_ode(vp)
    x0_2 = np.array([0.0, 0.0])
    P2 = poincare_section(f2, x0_2, vp.T, n_transient=200, n_points=50)
    lam2 = lyapunov_largest(f2, P2[-1], vp.T, n_steps=10000, dt=1e-3, delta0=1e-8)
    print("[Vehicle] Poincare last sample:", P2[-1])
    print("[Vehicle] crude largest Lyapunov ~", lam2, "1/s")

    # 3) CSTR
    cp = CSTRParams()
    f3 = cstr_ode(cp)
    x0_3 = np.array([0.9, 305.0])
    P3 = poincare_section(f3, x0_3, cp.T, n_transient=400, n_points=60)
    lam3 = lyapunov_largest(f3, P3[-1], cp.T, n_steps=12000, dt=2e-3, delta0=1e-8)
    print("[CSTR] Poincare last sample:", P3[-1])
    print("[CSTR] crude largest Lyapunov ~", lam3, "1/s")

if __name__ == "__main__":
    main()
