import numpy as np
import matplotlib.pyplot as plt
import control as ct  # pip install control

# Physical parameters (example)
m = 1000.0   # kg
b = 50.0     # N*s/m (effective drag)
Ku = 500.0   # N per unit control input (normalized throttle)

# Desired specs
Mp = 0.1       # max overshoot <= 10%
Ts = 5.0       # 2% settling time around 5 s

# Compute zeta from Mp (approximate inverse of Mp-zeta relation)
# Here we choose a typical damping ratio for 10% overshoot:
zeta = 0.6

# Natural frequency from settling time formula Ts ≈ 4/(zeta*wn)
wn = 4.0 / (zeta * Ts)

# PI gains from section 4 formulas
Kp = (2.0 * zeta * wn * m - b) / Ku
Ki = (wn**2 * m) / Ku

print("PI gains:")
print("Kp =", Kp)
print("Ki =", Ki)

# Define plant G(s) = Ku / (m s + b)
numG = [Ku]
denG = [m, b]
G = ct.TransferFunction(numG, denG)

# Controller C(s) = Kp + Ki/s
C = ct.TransferFunction([Kp, Ki], [1, 0])

# Closed-loop transfer function from reference to speed
T = ct.feedback(C * G, 1)

# Step response for a step in reference speed (1 m/s step)
t = np.linspace(0, 20, 1000)
t, y = ct.step_response(T, t)

plt.figure()
plt.plot(t, y)
plt.axhline(1.0, linestyle="--")
plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")
plt.title("Cruise Control: Closed-Loop Step Response (PI)")
plt.grid(True)
plt.show()
