import numpy as np

# Parameters
m = 2.0                 # mass (kg)
g = 9.81                # gravity (m/s^2)
k = 5000.0              # contact stiffness (N/m)
d = 50.0                # contact damping (N*s/m)
h = 1e-3                # time step (s)
T = 1.0                 # total time

# State: position q (m), velocity v (m/s)
q, v = 0.3, -1.0        # start above ground, moving down

def contact_force(q, v):
    # ground at q=0; gap g(q)=q
    if q >= 0:
        return 0.0
    # penalty: lambda_n = k(-q) - d v (only when penetrating)
    return k * (-q) - d * v

qs, vs, ts = [], [], []
t = 0.0
while t <= T:
    f_ext = -m*g + contact_force(q, v)
    a = f_ext / m

    # symplectic Euler
    v = v + h * a
    q = q + h * v

    qs.append(q); vs.append(v); ts.append(t)
    t += h

print("Final position, velocity:", q, v)
      
