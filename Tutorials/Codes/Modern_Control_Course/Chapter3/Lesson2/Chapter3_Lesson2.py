import numpy as np
from scipy.integrate import solve_ivp

# Example: time-varying linear system x_dot = A(t) x + b(t)
def A_of_t(t: float) -> np.ndarray:
    return np.array([[0.0, 1.0],
                     [-2.0 - 0.2*np.sin(t), -0.4]])

def b_of_t(t: float) -> np.ndarray:
    return np.array([0.0, 0.5*np.cos(t)])

def f(t: float, x: np.ndarray) -> np.ndarray:
    return A_of_t(t) @ x + b_of_t(t)

# Solve with SciPy
t0, tf = 0.0, 10.0
x0 = np.array([1.0, 0.0])
sol = solve_ivp(fun=f, t_span=(t0, tf), y0=x0, max_step=0.01, rtol=1e-8, atol=1e-10)

# From-scratch RK4 on the same system
def rk4_step(t: float, x: np.ndarray, h: float) -> np.ndarray:
    k1 = f(t, x)
    k2 = f(t + 0.5*h, x + 0.5*h*k1)
    k3 = f(t + 0.5*h, x + 0.5*h*k2)
    k4 = f(t + h,     x + h*k3)
    return x + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4)

h = 0.01
N = int((tf - t0)/h)
ts = np.linspace(t0, tf, N+1)
xs = np.zeros((N+1, 2))
xs[0] = x0
t = t0
for k in range(N):
    xs[k+1] = rk4_step(t, xs[k], h)
    t += h

print("SciPy final state:", sol.y[:, -1])
print("RK4  final state:", xs[-1])
      
