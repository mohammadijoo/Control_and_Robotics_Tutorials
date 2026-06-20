import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import StateSpace, lsim

A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0],
              [1.0]])
C = np.array([[1.0, 0.0]])
D = np.array([[0.0]])

# Define an input u(t): a unit step
def u_of_t(t):
    return 1.0

# State ODE: xdot = A x + B u
def f(t, x):
    u = u_of_t(t)
    return (A @ x + (B.flatten() * u))

t0, tf = 0.0, 5.0
x0 = np.array([0.5, 0.0])

sol = solve_ivp(f, (t0, tf), x0, dense_output=True, max_step=1e-2)

t = np.linspace(t0, tf, 1001)
x = sol.sol(t).T
u = np.ones_like(t)  # step input
y = (x @ C.T).flatten() + (D.flatten()[0] * u)

print("x(tf) =", x[-1])
print("y(tf) =", y[-1])

# Optional: use scipy.signal StateSpace for lsim (continuous-time)
sys = StateSpace(A, B, C, D)
tout, yout, xout = lsim(sys, U=u, T=t, X0=x0)
print("lsim y(tf) =", yout[-1])
      
