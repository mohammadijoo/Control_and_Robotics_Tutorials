# Chapter18_Lesson1.py
# Work, Energy, and Power in Mechanical, Electrical, Fluid, and Thermal Systems
# The script simulates four lumped dynamic models and checks energy balances.

import numpy as np

def trapz(y, t):
    return np.trapz(y, t)

t0, tf, dt = 0.0, 12.0, 1e-4
t = np.arange(t0, tf + dt, dt)
n = len(t)

# ==========================================================
# 1) Mechanical: mass-spring-damper
#    m xdd + c xd + k x = F(t)
# ==========================================================
m, c, k = 1.5, 0.8, 12.0

def F_in(tt):
    return 2.5 * np.sin(1.2 * tt) + 1.2 * np.cos(0.4 * tt)

x = np.zeros(n)
v = np.zeros(n)
Pm_in = np.zeros(n)
Pm_diss = np.zeros(n)
Em = np.zeros(n)

for kidx in range(n - 1):
    f = F_in(t[kidx])
    a = (f - c * v[kidx] - k * x[kidx]) / m
    v[kidx + 1] = v[kidx] + dt * a
    x[kidx + 1] = x[kidx] + dt * v[kidx]

    Pm_in[kidx] = f * v[kidx]
    Pm_diss[kidx] = c * v[kidx] ** 2
    Em[kidx] = 0.5 * m * v[kidx] ** 2 + 0.5 * k * x[kidx] ** 2

# last sample bookkeeping
Pm_in[-1] = F_in(t[-1]) * v[-1]
Pm_diss[-1] = c * v[-1] ** 2
Em[-1] = 0.5 * m * v[-1] ** 2 + 0.5 * k * x[-1] ** 2

Wm_in = trapz(Pm_in, t)
Wm_diss = trapz(Pm_diss, t)
mech_balance_error = Em[-1] - Em[0] - (Wm_in - Wm_diss)

# ==========================================================
# 2) Electrical: series RLC
#    L didt + R i + q/C = v_in(t),   dq/dt = i
# ==========================================================
R, L, C = 2.0, 0.6, 0.25

def v_in(tt):
    return 5.0 * np.sin(2.0 * tt) + 2.0 * np.cos(0.5 * tt)

q = np.zeros(n)   # capacitor charge
i = np.zeros(n)
Pe_in = np.zeros(n)
Pe_diss = np.zeros(n)
Ee = np.zeros(n)

for kidx in range(n - 1):
    vin = v_in(t[kidx])
    di = (vin - R * i[kidx] - q[kidx] / C) / L
    dq = i[kidx]

    i[kidx + 1] = i[kidx] + dt * di
    q[kidx + 1] = q[kidx] + dt * dq

    Pe_in[kidx] = vin * i[kidx]
    Pe_diss[kidx] = R * i[kidx] ** 2
    Ee[kidx] = 0.5 * L * i[kidx] ** 2 + 0.5 * (q[kidx] ** 2) / C

Pe_in[-1] = v_in(t[-1]) * i[-1]
Pe_diss[-1] = R * i[-1] ** 2
Ee[-1] = 0.5 * L * i[-1] ** 2 + 0.5 * (q[-1] ** 2) / C

We_in = trapz(Pe_in, t)
We_diss = trapz(Pe_diss, t)
elec_balance_error = Ee[-1] - Ee[0] - (We_in - We_diss)

# ==========================================================
# 3) Fluid (small-signal hydraulic chamber + line)
#    C_h dp/dt = q_in(t) - q
#    L_h dq/dt = p - R_h q
#    Power pair: p * q   (pressure * volumetric flow)
# ==========================================================
C_h, L_h, R_h = 0.08, 0.15, 1.7

def q_in(tt):
    return 0.8 * np.sin(0.9 * tt) + 0.2 * np.cos(0.3 * tt)

p = np.zeros(n)   # pressure deviation
qf = np.zeros(n)  # line flow
Pf_in = np.zeros(n)
Pf_diss = np.zeros(n)
Ef = np.zeros(n)

for kidx in range(n - 1):
    qsrc = q_in(t[kidx])
    dp = (qsrc - qf[kidx]) / C_h
    dqf = (p[kidx] - R_h * qf[kidx]) / L_h

    p[kidx + 1] = p[kidx] + dt * dp
    qf[kidx + 1] = qf[kidx] + dt * dqf

    Pf_in[kidx] = p[kidx] * qsrc
    Pf_diss[kidx] = R_h * qf[kidx] ** 2
    Ef[kidx] = 0.5 * C_h * p[kidx] ** 2 + 0.5 * L_h * qf[kidx] ** 2

Pf_in[-1] = p[-1] * q_in(t[-1])
Pf_diss[-1] = R_h * qf[-1] ** 2
Ef[-1] = 0.5 * C_h * p[-1] ** 2 + 0.5 * L_h * qf[-1] ** 2

Wf_in = trapz(Pf_in, t)
Wf_diss = trapz(Pf_diss, t)
fluid_balance_error = Ef[-1] - Ef[0] - (Wf_in - Wf_diss)

# ==========================================================
# 4) Thermal: lumped thermal capacitance with thermal resistance to ambient
#    C_th dT/dt = Q_in(t) - (T - T_env)/R_th
#    Heat flow Qdot has unit W (already power)
# ==========================================================
C_th, R_th, T_env = 600.0, 0.4, 293.15

def Q_in(tt):
    return 120.0 + 40.0 * np.sin(0.15 * tt)

T = np.ones(n) * T_env
Pt_in = np.zeros(n)
Pt_out = np.zeros(n)
Et = np.zeros(n)  # stored thermal energy relative to ambient

for kidx in range(n - 1):
    qdot_in = Q_in(t[kidx])
    qdot_out = (T[kidx] - T_env) / R_th
    dT = (qdot_in - qdot_out) / C_th

    T[kidx + 1] = T[kidx] + dt * dT

    Pt_in[kidx] = qdot_in
    Pt_out[kidx] = qdot_out
    Et[kidx] = C_th * (T[kidx] - T_env)

Pt_in[-1] = Q_in(t[-1])
Pt_out[-1] = (T[-1] - T_env) / R_th
Et[-1] = C_th * (T[-1] - T_env)

Wt_in = trapz(Pt_in, t)
Wt_out = trapz(Pt_out, t)
thermal_balance_error = Et[-1] - Et[0] - (Wt_in - Wt_out)

print("=== Energy Balance Audit (explicit-Euler residuals (shrink as dt decreases)) ===")
print(f"Mechanical residual  : {mech_balance_error:.6e} J")
print(f"Electrical residual  : {elec_balance_error:.6e} J")
print(f"Fluid residual       : {fluid_balance_error:.6e} (energy units)")
print(f"Thermal residual     : {thermal_balance_error:.6e} J")
print()
print("Final stored energies:")
print(f"E_mech(tf) = {Em[-1]:.6f}")
print(f"E_elec(tf) = {Ee[-1]:.6f}")
print(f"E_fluid(tf)= {Ef[-1]:.6f}")
print(f"E_therm(tf)= {Et[-1]:.6f}")
