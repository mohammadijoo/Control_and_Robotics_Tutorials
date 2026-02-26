import numpy as np

# Closed-loop first-order model:
# y_{k+1} = a * y_k + b * r_k, with r_k = 1 (unit step)
def simulate_step(Kp, K=1.0, tau=0.5, dt=0.01, T=5.0):
    a = np.exp(-dt * (1.0 + Kp * K) / tau)
    b = (1.0 - a)  # ensures steady-state gain 1
    n = int(T / dt)
    y = np.zeros(n)
    r = 1.0
    for k in range(n - 1):
        y[k+1] = a * y[k] + b * r
    t = np.linspace(0.0, T, n)
    return t, y

def metrics(t, y, tol=0.02):
    y_inf = y[-1]
    e_inf = 1.0 - y_inf
    Mp = 100.0 * (np.max(y) - y_inf) / max(y_inf, 1e-9)
    # settling time: first time from which we remain in 2% band
    idx = np.where(np.abs(y - y_inf) > tol * abs(y_inf))[0]
    Ts = t[idx[-1]] if idx.size > 0 else 0.0
    return e_inf, Mp, Ts

Kp_candidates = [0.5, 1.0, 2.0, 4.0]
for Kp in Kp_candidates:
    t, y = simulate_step(Kp)
    e_inf, Mp, Ts = metrics(t, y)
    print(f"Kp={Kp:.2f}: e_inf={e_inf:.3f}, Mp={Mp:.1f}%, Ts={Ts:.2f}s")
      
