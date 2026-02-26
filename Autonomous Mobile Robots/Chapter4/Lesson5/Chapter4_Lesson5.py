# Chapter4_Lesson5.py
# Parameter Effects on Real Navigation (Mobile Robot Dynamics - Applied)
#
# This script simulates a differential-drive ground robot with a simple
# longitudinal + yaw-rate dynamic model and shows how physical parameters
# (mass, yaw inertia, wheel radius, wheelbase, friction) affect trajectory
# tracking under a nominal PI controller.
#
# Dependencies: numpy, matplotlib (optional)

import numpy as np

# -----------------------------
# Model and simulation utilities
# -----------------------------
class Params:
    def __init__(self, m=25.0, Iz=2.0, b=0.45, r=0.10, mu=0.8, cv=0.4, cw=0.6, g=9.81):
        self.m = float(m)     # mass [kg]
        self.Iz = float(Iz)   # yaw inertia [kg*m^2]
        self.b = float(b)     # wheel track (left-right distance) [m]
        self.r = float(r)     # wheel radius [m]
        self.mu = float(mu)   # Coulomb friction coefficient (flat ground)
        self.cv = float(cv)   # linear damping in v channel [1/s]
        self.cw = float(cw)   # linear damping in omega channel [1/s]
        self.g = float(g)

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def wheel_torques_from_FM(F, Mz, p: Params, tau_max=35.0):
    # Solve:
    # F = (tauR + tauL)/r
    # M = (b/(2r))*(tauR - tauL)
    tauR = 0.5 * p.r * (F + 2.0 * Mz / p.b)
    tauL = 0.5 * p.r * (F - 2.0 * Mz / p.b)
    tauR = clamp(tauR, -tau_max, tau_max)
    tauL = clamp(tauL, -tau_max, tau_max)
    return tauL, tauR

def dynamics(x, tauL, tauR, p: Params):
    # State x = [px, py, theta, v, omega]
    px, py, th, v, om = x

    # Effective force and yaw moment from wheel torques
    F = (tauR + tauL) / p.r
    Mz = (p.b / (2.0 * p.r)) * (tauR - tauL)

    # Traction limit (simple)
    Fmax = p.mu * p.m * p.g
    F = clamp(F, -Fmax, Fmax)

    # Dynamics
    vdot = (1.0 / p.m) * F - p.cv * v
    omdot = (1.0 / p.Iz) * Mz - p.cw * om

    # Kinematics
    pxdot = v * np.cos(th)
    pydot = v * np.sin(th)
    thdot = om

    return np.array([pxdot, pydot, thdot, vdot, omdot], dtype=float)

def rk4_step(x, u, dt, p: Params):
    tauL, tauR = u
    k1 = dynamics(x, tauL, tauR, p)
    k2 = dynamics(x + 0.5 * dt * k1, tauL, tauR, p)
    k3 = dynamics(x + 0.5 * dt * k2, tauL, tauR, p)
    k4 = dynamics(x + dt * k3, tauL, tauR, p)
    return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

# -----------------------------
# Reference command and controller
# -----------------------------
def reference(t):
    # A smooth, time-varying command (keeps within typical indoor AMR limits)
    v_ref = 0.9 + 0.2 * np.sin(0.2 * t)               # m/s
    om_ref = 0.35 + 0.15 * np.sin(0.17 * t + 0.7)     # rad/s
    return v_ref, om_ref

class PIController:
    def __init__(self, p_nom: Params, kvp=180.0, kvi=35.0, kwp=18.0, kwi=4.0):
        self.p = p_nom
        self.kvp, self.kvi = kvp, kvi
        self.kwp, self.kwi = kwp, kwi
        self.iv = 0.0
        self.iw = 0.0

    def reset(self):
        self.iv = 0.0
        self.iw = 0.0

    def __call__(self, x, v_ref, om_ref, dt):
        v = x[3]
        om = x[4]
        ev = v_ref - v
        ew = om_ref - om
        self.iv += ev * dt
        self.iw += ew * dt

        # Desired generalized force and yaw moment (computed using nominal parameters)
        F_cmd = self.p.m * (self.kvp * ev + self.kvi * self.iv) + self.p.m * self.p.cv * v
        M_cmd = self.p.Iz * (self.kwp * ew + self.kwi * self.iw) + self.p.Iz * self.p.cw * om

        tauL, tauR = wheel_torques_from_FM(F_cmd, M_cmd, self.p)
        return tauL, tauR

# -----------------------------
# Experiments
# -----------------------------
def simulate(p_true: Params, p_nom: Params, T=35.0, dt=0.01):
    ctrl = PIController(p_nom)
    N = int(T/dt) + 1
    x = np.zeros(5)  # start at origin, zero velocities
    xs = np.zeros((N, 5))
    refs = np.zeros((N, 2))
    ts = np.linspace(0.0, T, N)

    for i, t in enumerate(ts):
        v_ref, om_ref = reference(t)
        refs[i] = [v_ref, om_ref]

        tauL, tauR = ctrl(x, v_ref, om_ref, dt)
        x = rk4_step(x, (tauL, tauR), dt, p_true)
        xs[i] = x
    return ts, xs, refs

def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))

def main():
    p_nom = Params()  # controller assumes these are true
    # Baseline truth equals nominal
    ts0, xs0, _ = simulate(p_nom, p_nom)

    # Parameter sweeps: each case changes ONE parameter in the true plant
    sweep = [
        ("mass_m",       [15.0, 25.0, 40.0], "kg"),
        ("yaw_Iz",       [1.2, 2.0, 3.5],     "kg*m^2"),
        ("wheel_r",      [0.095, 0.10, 0.105],"m"),
        ("wheelbase_b",  [0.40, 0.45, 0.52],  "m"),
        ("friction_mu",  [0.5, 0.8, 1.0],     "-"),
    ]

    results = []
    for name, values, unit in sweep:
        for v in values:
            p_true = Params()
            setattr(p_true, "m" if name=="mass_m" else
                           "Iz" if name=="yaw_Iz" else
                           "r" if name=="wheel_r" else
                           "b" if name=="wheelbase_b" else
                           "mu", float(v))

            ts, xs, _ = simulate(p_true, p_nom)

            # Compare to baseline (nominal) trajectory at the same time indices
            pos_err = np.linalg.norm(xs[:, :2] - xs0[:, :2], axis=1)
            th_err  = np.unwrap(xs[:, 2]) - np.unwrap(xs0[:, 2])
            v_err   = xs[:, 3] - xs0[:, 3]
            w_err   = xs[:, 4] - xs0[:, 4]

            results.append({
                "parameter": name,
                "value": v,
                "unit": unit,
                "rms_pos_m": rms(pos_err),
                "rms_theta_rad": rms(th_err),
                "rms_v_mps": rms(v_err),
                "rms_omega_rps": rms(w_err),
            })

    # Print a compact table
    print("Parameter sweep results (true plant varies; controller stays nominal):")
    for r in results:
        print(f"{r['parameter']:12s}={r['value']:7.3f} {r['unit']:7s} | "
              f"pos_rms={r['rms_pos_m']:.3f} m, "
              f"theta_rms={r['rms_theta_rad']:.3f} rad")

    # Optional: save CSV
    try:
        import csv
        with open("Chapter4_Lesson5_results.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)
        print("Saved: Chapter4_Lesson5_results.csv")
    except Exception as e:
        print("CSV save skipped:", e)

if __name__ == "__main__":
    main()
