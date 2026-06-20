import numpy as np

# Physical parameters (mass-spring-damper)
m = 1.0
b = 0.4
k = 4.0

# States: x1 = q (position), x2 = qdot (velocity)
A = np.array([[0.0, 1.0],
              [-k/m, -b/m]])
B = np.array([[0.0],
              [1.0/m]])
C = np.array([[1.0, 0.0]])   # output position
D = np.array([[0.0]])

# Simulation using SciPy
from scipy.integrate import solve_ivp

def f(t, x, u_func):
    u = u_func(t)
    return (A @ x + (B.flatten() * u))

# Step input u(t) = 1
u_func = lambda t: 1.0

t_span = (0.0, 10.0)
x0 = np.array([0.0, 0.0])  # q(0)=0, qdot(0)=0
sol = solve_ivp(lambda t, x: f(t, x, u_func), t_span, x0, max_step=0.01)

t = sol.t
x = sol.y
y = (C @ x).flatten()

print("Final state:", x[:, -1])
print("Final output y (position):", y[-1])

# Optional: python-control for state-space object
try:
    import control as ct
    sys = ct.ss(A, B, C, D)
    # Step response
    t2, y2 = ct.step_response(sys, T=np.linspace(0, 10, 2001))
    print("python-control step y(t) sample:", y2[:5])
except Exception as e:
    print("python-control not available or failed to import:", e)
      
