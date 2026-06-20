import numpy as np

def f_oscillator(t, y, k=10.0):
    q, qdot = y
    dqdt = qdot
    dqdotdt = -k * q
    return np.array([dqdt, dqdotdt])

def rk4_step(f, t, y, h, **kwargs):
    k1 = f(t,         y,             **kwargs)
    k2 = f(t + 0.5*h, y + 0.5*h*k1,  **kwargs)
    k3 = f(t + 0.5*h, y + 0.5*h*k2,  **kwargs)
    k4 = f(t + h,     y + h*k3,      **kwargs)
    return y + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4)

t = 0.0
y = np.array([0.1, 0.0])  # small initial displacement
h = 0.001
T = 1.0
while t < T:
    y = rk4_step(f_oscillator, t, y, h, k=10.0)
    t += h
# y now approximates [q(T), qdot(T)]
print("q(T), qdot(T) =", y)
      
