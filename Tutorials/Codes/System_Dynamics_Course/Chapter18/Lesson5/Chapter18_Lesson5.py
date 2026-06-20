# Chapter18_Lesson5.py
# Port-Hamiltonian mechatronic model (electrical + rotational + elastic)
import numpy as np
import matplotlib.pyplot as plt

L, R_a = 0.35, 1.8
J, b = 0.02, 0.08
k, Kt = 4.0, 0.22

Jm = np.array([[0.0, Kt, 0.0], [-Kt, 0.0, -1.0], [0.0, 1.0, 0.0]])
Rm = np.diag([R_a, b, 0.0])
G = np.array([1.0, 0.0, 0.0])

def gradH(x):
    phi, p, q = x
    return np.array([phi/L, p/J, k*q])

def H(x):
    phi, p, q = x
    return 0.5*phi*phi/L + 0.5*p*p/J + 0.5*k*q*q

def u(t):
    return 8.0 if t < 0.8 else (3.0 if t < 1.6 else 0.0)

def f(t, x):
    return (Jm - Rm) @ gradH(x) + G * u(t)

def rk4(t, x, h):
    k1 = f(t, x)
    k2 = f(t + 0.5*h, x + 0.5*h*k1)
    k3 = f(t + 0.5*h, x + 0.5*h*k2)
    k4 = f(t + h, x + h*k3)
    return x + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4)

t0, tf, h = 0.0, 4.0, 1e-3
t = np.arange(t0, tf + h, h)
x = np.zeros((len(t), 3))
Hhist = np.zeros(len(t)); Pin = np.zeros(len(t)); Pdiss = np.zeros(len(t))
for n in range(len(t)-1):
    g = gradH(x[n]); i, w = g[0], g[1]
    Hhist[n] = H(x[n]); Pin[n] = u(t[n])*i; Pdiss[n] = R_a*i*i + b*w*w
    x[n+1] = rk4(t[n], x[n], h)
g = gradH(x[-1]); Hhist[-1] = H(x[-1]); Pin[-1] = u(t[-1])*g[0]; Pdiss[-1] = R_a*g[0]**2 + b*g[1]**2

lhs = Hhist[-1] - Hhist[0]
rhs = np.trapz(Pin - Pdiss, t)
print("Energy residual =", lhs - rhs)

np.savetxt("Chapter18_Lesson5_python_results.csv",
           np.column_stack([t, x, Hhist, Pin, Pdiss]),
           delimiter=",",
           header="t,phi,p,q,H,Pin,Pdiss", comments="")

plt.figure()
plt.plot(t, x[:,0]/L, label="i(t)")
plt.plot(t, x[:,1]/J, label="omega(t)")
plt.plot(t, x[:,2], label="q(t)")
plt.grid(True); plt.legend(); plt.xlabel("t (s)")
plt.title("Port-Hamiltonian Mechatronic States")
plt.show()
