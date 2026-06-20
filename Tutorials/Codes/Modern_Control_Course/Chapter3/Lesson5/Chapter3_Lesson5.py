import numpy as np

# Time-varying linear system: xdot = A(t)x + b(t)
def A(t):
    # bounded, continuous A(t) on any finite interval
    return np.array([[0.0, 1.0],
                     [-2.0 - 0.5*np.sin(t), -0.4]])

def b(t):
    return np.array([0.0, 1.0*np.cos(2.0*t)])

def f(t, x):
    return A(t) @ x + b(t)

t0, T = 0.0, 8.0
x0 = np.array([1.0, 0.0])

# 1) Reference solution with SciPy's solve_ivp (adaptive RK method)
from scipy.integrate import solve_ivp
sol = solve_ivp(lambda t, x: f(t, x), (t0, T), x0, method="RK45", rtol=1e-9, atol=1e-12)
t_ref = sol.t
x_ref = sol.y.T

# 2) From-scratch explicit Euler method with uniform step
def euler_uniform(f, t0, T, x0, N):
    h = (T - t0) / N
    t = t0
    x = x0.astype(float).copy()
    traj_t = [t]
    traj_x = [x.copy()]
    for k in range(N):
        x = x + h * f(t, x)
        t = t + h
        traj_t.append(t)
        traj_x.append(x.copy())
    return np.array(traj_t), np.array(traj_x)

# Compare Euler with decreasing step sizes against reference (interpolate reference)
from scipy.interpolate import interp1d
x_ref_interp = interp1d(t_ref, x_ref, axis=0, kind="cubic", fill_value="extrapolate")

for N in [200, 400, 800, 1600]:
    t_eu, x_eu = euler_uniform(f, t0, T, x0, N)
    err = np.max(np.linalg.norm(x_eu - x_ref_interp(t_eu), axis=1))
    print(f"N={N:4d}, h={(T-t0)/N:.5f}, max error ~ {err:.3e}")

# Uniqueness sanity check: same IVP should yield same trajectory (within numerical tolerances)
sol2 = solve_ivp(lambda t, x: f(t, x), (t0, T), x0, method="DOP853", rtol=1e-10, atol=1e-13)
x2_interp = interp1d(sol2.t, sol2.y.T, axis=0, kind="cubic", fill_value="extrapolate")
grid = np.linspace(t0, T, 401)
diff = np.max(np.linalg.norm(x_ref_interp(grid) - x2_interp(grid), axis=1))
print("Max difference between two high-accuracy solvers:", diff)
      
