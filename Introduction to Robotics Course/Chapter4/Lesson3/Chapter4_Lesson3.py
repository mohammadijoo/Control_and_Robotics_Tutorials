
import numpy as np

# --- Power budget & runtime ---
Vb = 14.8            # battery voltage [V]
C_Ah = 5.0           # capacity [Ah]
eta = 0.9            # overall conversion efficiency

# loads: (V_i, avg current)
loads = [
    (14.8, 2.0),     # motors (avg)
    (5.0,  1.2),     # computer rail
    (3.3,  0.5)      # sensors rail
]
P_tot = sum(V * I for V, I in loads)          # [W]
E_b = 3600.0 * Vb * C_Ah                      # [J]
T_run = eta * E_b / P_tot / 3600.0            # [hours]

print("Average power [W] =", P_tot)
print("Estimated runtime [h] =", T_run)

# --- Wire voltage drop ---
rho = 1.68e-8        # copper resistivity [ohm*m]
L = 1.5              # one-way length [m]
A = 1.0e-6           # area [m^2] ~ AWG18 order
I = 5.0              # current [A]
Rw = rho * L / A
dV = I * Rw

print("Wire resistance [ohm] =", Rw)
print("Voltage drop [V] =", dV)

# --- RC cutoff ---
R = 1e3              # [ohm]
C = 0.1e-6           # [F]
fc = 1.0 / (2*np.pi*R*C)
print("Cutoff frequency [Hz] =", fc)
      