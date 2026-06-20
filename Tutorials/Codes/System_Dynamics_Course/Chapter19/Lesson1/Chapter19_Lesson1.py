# Chapter19_Lesson1.py
# 1D heat/diffusion and wave equations (explicit finite differences)

import numpy as np
import matplotlib.pyplot as plt

# --- Heat / diffusion: u_t = alpha * u_xx ---
L = 1.0
alpha = 0.2
Nx = 81
x = np.linspace(0.0, L, Nx)
dx = x[1] - x[0]
r = 0.45  # stability: r <= 0.5
dt = r * dx * dx / alpha
T = 0.25
Nt = int(T / dt)

u = np.sin(np.pi * x) + 0.2 * np.sin(3 * np.pi * x)
u[0] = 0.0
u[-1] = 0.0
heat_snaps = [u.copy()]
heat_times = [0.0]

for n in range(Nt):
    un = u.copy()
    u[1:-1] = un[1:-1] + alpha * dt / dx**2 * (un[2:] - 2 * un[1:-1] + un[:-2])
    u[0] = 0.0
    u[-1] = 0.0
    if n % max(1, Nt // 4) == 0:
        heat_snaps.append(u.copy())
        heat_times.append((n + 1) * dt)

# --- Wave: u_tt = c^2 * u_xx ---
c = 1.0
Nxw = 201
xw = np.linspace(0.0, L, Nxw)
dxw = xw[1] - xw[0]
s = 0.95  # CFL: s <= 1
dtw = s * dxw / c
Tw = 1.0
Ntw = int(Tw / dtw)

u0 = np.exp(-180 * (xw - 0.35) ** 2)  # initial displacement
v0 = np.zeros_like(xw)                 # initial velocity
u0[0] = 0.0
u0[-1] = 0.0

u_prev = u0.copy()
u_curr = u0.copy()
u_curr[1:-1] = (
    u0[1:-1]
    + dtw * v0[1:-1]
    + 0.5 * (c * dtw / dxw) ** 2 * (u0[2:] - 2 * u0[1:-1] + u0[:-2])
)
u_curr[0] = 0.0
u_curr[-1] = 0.0

wave_snaps = [u_prev.copy(), u_curr.copy()]
wave_times = [0.0, dtw]

for n in range(1, Ntw):
    u_next = np.zeros_like(u_curr)
    u_next[1:-1] = (
        2 * u_curr[1:-1] - u_prev[1:-1]
        + (c * dtw / dxw) ** 2 * (u_curr[2:] - 2 * u_curr[1:-1] + u_curr[:-2])
    )
    u_prev, u_curr = u_curr, u_next
    if n % max(1, Ntw // 4) == 0:
        wave_snaps.append(u_curr.copy())
        wave_times.append((n + 1) * dtw)

# Plot snapshots
plt.figure(figsize=(8, 4))
for k, t in enumerate(heat_times):
    plt.plot(x, heat_snaps[k], label=f"t={t:.3f}")
plt.title("Heat / Diffusion Equation")
plt.xlabel("x")
plt.ylabel("u(x,t)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

plt.figure(figsize=(8, 4))
for k, t in enumerate(wave_times[:5]):
    plt.plot(xw, wave_snaps[k], label=f"t={t:.3f}")
plt.title("Wave Equation")
plt.xlabel("x")
plt.ylabel("u(x,t)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()
