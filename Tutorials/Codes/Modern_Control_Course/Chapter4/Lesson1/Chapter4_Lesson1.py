import numpy as np
from scipy.integrate import solve_ivp

# System: xdot = -x + u(t), y = x
def u(t):
    # Example input: unit step
    return 1.0 if t >= 0.0 else 0.0

def f(t, x):
    return -x[0] + u(t)

t0, tf = 0.0, 5.0
x0 = np.array([0.2])

sol = solve_ivp(lambda t, x: [f(t, x)], (t0, tf), x0, dense_output=True, max_step=0.01)
t = np.linspace(t0, tf, 501)
x = sol.sol(t)[0]
y = x.copy()

print("x(tf) =", x[-1])
print("y(tf) =", y[-1])

# Note (control-oriented libraries):
# - scipy.signal and the 'control' package can represent state-space models directly,
#   but the explicit (A,B,C,D) representation is introduced in Lesson 2.
