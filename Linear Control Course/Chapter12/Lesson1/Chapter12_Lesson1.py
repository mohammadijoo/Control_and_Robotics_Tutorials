import numpy as np
import control as ct
import matplotlib.pyplot as plt

# --- Specifications ---
Mp_star = 0.10      # 10% overshoot (relative)
Ts_star = 2.0       # 2% settling time [s]
alpha   = 4.0       # third pole factor p3 = alpha * zeta * omega_n

# --- Step 1: Compute zeta from Mp* ---
lnMp = np.log(Mp_star)
zeta = -lnMp / np.sqrt(np.pi**2 + lnMp**2)

# --- Step 2: Compute omega_n from Ts* ---
omega_n = 4.0 / (zeta * Ts_star)

# --- Step 3: Choose p3 and compute PID gains ---
p3 = alpha * zeta * omega_n

Kd = 2*zeta*omega_n + p3 - 1.0
Kp = omega_n**2 + 2*zeta*omega_n*p3
Ki = omega_n**2 * p3

print(f"Computed gains: Kp={Kp:.3f}, Ki={Ki:.3f}, Kd={Kd:.3f}")

# --- Step 4: Build plant and controller ---
# Plant G(s) = 1/(s*(s+1))
s = ct.TransferFunction.s
G = 1 / (s*(s + 1))

# PID controller in parallel form: C(s) = Kp + Ki/s + Kd*s
C = Kp + Ki/s + Kd*s

# Closed-loop system under unity feedback
T = ct.feedback(C*G, 1)

# --- Step 5: Simulate step response ---
t = np.linspace(0, 10, 1000)
t_out, y_out = ct.step_response(T, t)

# Compute overshoot and settling time empirically
y_final = y_out[-1]
Mp_actual = (np.max(y_out) - y_final) / y_final
# 2% settling time
within = np.where(np.abs(y_out - y_final) < 0.02 * np.abs(y_final))[0]
Ts_actual = t_out[within[0]] if len(within) > 0 else np.nan

print(f"Approx. actual Mp={Mp_actual*100:.1f}%")
print(f"Approx. actual Ts={Ts_actual:.2f} s")

plt.figure()
plt.plot(t_out, y_out)
plt.axhline(y_final, linestyle="--")
plt.xlabel("t [s]")
plt.ylabel("y(t)")
plt.title("Closed-loop Step Response with Designed PID")
plt.grid(True)
plt.show()
