import numpy as np

# Bus parameters
C_bus = 3e-3       # 3 mF bus capacitor
R_bus = 0.08       # equivalent bus resistance (ohms)
V_oc  = 24.0       # open-circuit supply voltage
R_int = 0.12       # internal resistance (ohms)

Vmin = 20.0        # brownout threshold

# Load profile: base + step at t=0.5s
def I_load(t):
    base = 2.0      # A: logic + sensors
    step = 8.0 if t >= 0.5 else 0.0  # A: motors start
    return base + step

# Simple explicit Euler integration
dt = 1e-4
T  = 1.5
ts = np.arange(0, T, dt)
V  = np.zeros_like(ts)
V[0] = V_oc

for k in range(len(ts)-1):
    Il = I_load(ts[k])
    # supply current governed by internal resistance
    Isup = max((V_oc - V[k]) / R_int, 0.0)
    dVdt = (Isup - Il - V[k]/R_bus) / C_bus
    V[k+1] = V[k] + dt*dVdt

t_brown = ts[V < Vmin]
print("Brownout occurred?" , len(t_brown) > 0)
if len(t_brown) > 0:
    print("First brownout time:", t_brown[0])

# Energy estimate from power budget
rails = {
    "motors": {"V": 24.0, "I_nom": 3.5, "duty": 0.4},
    "logic" : {"V": 5.0,  "I_nom": 1.2, "duty": 1.0},
    "sbc"   : {"V": 12.0, "I_nom": 1.8, "duty": 0.6},
}
eta = {"motors": 0.95, "logic": 0.9, "sbc": 0.92}

P_avg = 0.0
for name, r in rails.items():
    Pj = r["V"] * r["I_nom"] * r["duty"]
    P_avg += Pj / eta[name]
print("Average bus power (W):", P_avg)
