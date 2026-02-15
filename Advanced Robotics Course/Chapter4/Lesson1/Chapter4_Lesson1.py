import casadi as ca
import numpy as np

# Discretization
N = 40
T = 2.0
h = T / N

nx = 2  # [p, v]
nu = 1  # [a]

# Continuous-time dynamics f(x,u)
x = ca.MX.sym("x", nx)
u = ca.MX.sym("u", nu)

p = x[0]
v = x[1]
a = u[0]

f = ca.vertcat(v, a)  # x_dot = [v; a]

# Decision variables
X = ca.MX.sym("X", nx, N + 1)  # states over horizon
U = ca.MX.sym("U", nu, N)      # controls over horizon

w   = []  # stacked decision vars
lbw = []  # lower bounds
ubw = []  # upper bounds
g   = []  # constraint expressions
lbg = []  # lower bounds on constraints
ubg = []  # upper bounds on constraints

J = 0  # objective

# Boundary conditions
p0 = 0.0
v0 = 0.0
pf = 1.0
vf = 0.0
amax = 2.0

# Initial state fixed
w.append(X[:, 0])
lbw += [p0, v0]
ubw += [p0, v0]

# Loop over horizon
for k in range(N):
    # Control at step k
    w.append(U[:, k])
    lbw += [-amax]
    ubw += [ amax]

    # Cost: 0.5 * a^2 * h
    J = J + 0.5 * h * ca.mtimes(U[:, k].T, U[:, k])

    # State at step k
    xk = X[:, k]

    # Integrate dynamics with forward Euler
    x_next = xk + h * f.subs({x: xk, u: U[:, k]})

    # Add next-state variable
    w.append(X[:, k + 1])
    # No bound yet; we will set for final state after the loop
    lbw += [-ca.inf, -ca.inf]
    ubw += [ ca.inf,  ca.inf]

    # Dynamics equality constraint
    g.append(X[:, k + 1] - x_next)
    lbg += [0.0, 0.0]
    ubg += [0.0, 0.0]

# Terminal state fixed
lbw[-2] = pf
ubw[-2] = pf
lbw[-1] = vf
ubw[-1] = vf

# Stack decision variables and constraints
w = ca.vertcat(*[ca.reshape(wi, -1, 1) for wi in w])
g = ca.vertcat(*g)

prob = {"f": J, "x": w, "g": g}
solver = ca.nlpsol("solver", "ipopt", prob,
                   {"ipopt.print_level": 0, "print_time": 0})

sol = solver(lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol["x"].full().flatten()

print("Optimal cost:", float(sol["f"]))
      
