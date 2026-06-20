# Chapter18_Lesson4.py
# Bond graph simulation: Se - 1 - (I,R,C) for a mass-spring-damper

import numpy as np
import matplotlib.pyplot as plt

def Se(t):
    return 2.0*np.sin(2*np.pi*0.8*t) + (1.0 if t >= 1.0 else 0.0)

def f_bg(t, x, m, b, C):
    q, p = x                     # states: q (C-state), p (I-state)
    v = p / m                    # common flow at 1-junction
    eR = b * v
    eC = q / C                   # = k q because C = 1/k
    qdot = v
    pdot = Se(t) - eR - eC       # 1-junction effort balance
    return np.array([qdot, pdot], dtype=float)

def rk4(fun, t, x, h, *params):
    k1 = fun(t, x, *params)
    k2 = fun(t + h/2, x + h*k1/2, *params)
    k3 = fun(t + h/2, x + h*k2/2, *params)
    k4 = fun(t + h, x + h*k3, *params)
    return x + (h/6)*(k1 + 2*k2 + 2*k3 + k4)

def main():
    m = 1.5
    k = 12.0
    b = 1.2
    C = 1.0 / k

    h = 1e-3
    t = np.arange(0.0, 10.0 + h, h)
    x = np.zeros((len(t), 2))
    x[0] = [0.10, 0.0]  # q(0), p(0)

    for i in range(len(t)-1):
        x[i+1] = rk4(f_bg, t[i], x[i], h, m, b, C)

    q = x[:, 0]
    p = x[:, 1]
    v = p / m
    eS = np.array([Se(ti) for ti in t])

    H = 0.5*p**2/m + 0.5*q**2/C
    Pdiss = b*v**2
    Ediss = np.cumsum(Pdiss) * h
    dH = np.gradient(H, h)
    residual = eS*v - Pdiss - dH

    print('Max |power residual| =', float(np.max(np.abs(residual))))

    np.savetxt(
        'Chapter18_Lesson4_python_output.csv',
        np.c_[t, q, v, H, Ediss, residual],
        delimiter=',',
        header='t,q,v,H,Ediss,residual',
        comments=''
    )

    plt.figure()
    plt.plot(t, q, label='q(t)')
    plt.plot(t, v, label='v(t)')
    plt.grid(True); plt.legend(); plt.title('States (bond-graph form)')

    plt.figure()
    plt.plot(t, H, label='Stored energy H')
    plt.plot(t, Ediss, label='Dissipated energy')
    plt.grid(True); plt.legend(); plt.title('Energy accounting')

    plt.figure()
    plt.plot(t, residual)
    plt.grid(True); plt.title('Power-balance residual')
    plt.show()

if __name__ == '__main__':
    main()
