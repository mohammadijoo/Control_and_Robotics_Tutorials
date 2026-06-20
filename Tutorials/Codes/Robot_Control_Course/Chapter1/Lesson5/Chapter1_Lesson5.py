
import numpy as np
from dataclasses import dataclass
from typing import Tuple
from scipy import signal

@dataclass
class SecondOrderSpecs:
    zeta: float
    omega_n: float

@dataclass
class StepMetrics:
    Mp: float          # percent overshoot
    Ts: float          # settling time (2 percent band)
    Tr: float          # rise time (10-90 percent)
    ess: float         # steady-state error
    J_ise: float       # integral squared error
    J_u: float         # control energy (for simple PD u)

def second_order_tf(specs: SecondOrderSpecs) -> signal.TransferFunction:
    zeta = specs.zeta
    wn = specs.omega_n
    num = [wn**2]
    den = [1.0, 2.0 * zeta * wn, wn**2]
    return signal.TransferFunction(num, den)

def simulate_step(specs: SecondOrderSpecs, t_final: float = 10.0, dt: float = 1e-3
                  ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    sys = second_order_tf(specs)
    t = np.arange(0.0, t_final, dt)
    tout, y = signal.step(sys, T=t)
    r = np.ones_like(tout)  # unit step reference
    return tout, r, y

def compute_step_metrics(t: np.ndarray, r: np.ndarray, y: np.ndarray,
                         zeta: float, omega_n: float) -> StepMetrics:
    # Percent overshoot
    y_inf = y[-1]
    y_max = np.max(y)
    Mp = 100.0 * max(0.0, (y_max - y_inf) / max(1e-9, y_inf))

    # Settling time: first time after which response stays within 2 percent
    tol = 0.02 * abs(y_inf)
    Ts = t[-1]
    for k in range(len(t)):
        if np.all(np.abs(y[k:] - y_inf) <= tol):
            Ts = t[k]
            break

    # Rise time 10-90 percent
    y10 = 0.1 * y_inf
    y90 = 0.9 * y_inf
    t10 = t[0]
    t90 = t[-1]
    for k in range(len(t)):
        if y[k] >= y10:
            t10 = t[k]
            break
    for k in range(len(t)):
        if y[k] >= y90:
            t90 = t[k]
            break
    Tr = t90 - t10

    # Steady-state error
    ess = r[-1] - y_inf

    # Integral squared error
    e = r - y
    J_ise = np.trapz(e * e, t)

    # Simple PD model: u = Kp * e + Kd * de/dt with gains chosen
    # to match zeta and omega_n for a unit-inertia joint: J = 1
    # Then Kp = omega_n**2, Kd = 2*zeta*omega_n
    Kp = omega_n**2
    Kd = 2.0 * zeta * omega_n
    de = np.gradient(e, t)
    u = Kp * e + Kd * de
    J_u = np.trapz(u * u, t)

    return StepMetrics(Mp=Mp, Ts=Ts, Tr=Tr, ess=ess, J_ise=J_ise, J_u=J_u)

if __name__ == "__main__":
    specs = SecondOrderSpecs(zeta=0.7, omega_n=8.0)
    t, r, y = simulate_step(specs, t_final=5.0, dt=1e-3)
    metrics = compute_step_metrics(t, r, y, specs.zeta, specs.omega_n)
    print(metrics)
