import numpy as np
from scipy.optimize import minimize

# Problem parameters
N = 40          # number of intervals
T = 1.0
h = T / N
n_x = 2
n_u = 1

def unpack(z):
    """Split decision vector into X (states) and U (controls)."""
    X_flat = z[: (N + 1) * n_x]
    U_flat = z[(N + 1) * n_x :]
    X = X_flat.reshape((N + 1, n_x))
    U = U_flat.reshape((N, n_u))
    return X, U

def objective(z):
    X, U = unpack(z)
    # Integral approximation J = sum h * u^2
    return h * np.sum(U[:, 0] ** 2)

def constraints(z):
    X, U = unpack(z)
    cons = []

    # Initial state constraints: x_0 = (0, 0)
    cons.extend(X[0] - np.array([0.0, 0.0]))

    # Terminal state constraints: x_N = (1, 0)
    cons.extend(X[-1] - np.array([1.0, 0.0]))

    # Trapezoidal collocation constraints
    for k in range(N):
        xk = X[k]
        xkp1 = X[k + 1]
        uk = U[k, 0]

        # Double integrator dynamics: dot p = v, dot v = u
        fk = np.array([xk[1], uk])
        # piecewise constant control, approximate f at k+1 with same uk
        fkp1 = np.array([xkp1[1], uk])

        cons.extend(xkp1 - xk - 0.5 * h * (fk + fkp1))

    return np.array(cons)

# Initial guess: straight-line in position, zero velocity and control
z0 = np.zeros((N + 1) * n_x + N * n_u)
X0 = np.zeros((N + 1, n_x))
for k in range(N + 1):
    t = k * h
    X0[k, 0] = t
z0[: (N + 1) * n_x] = X0.reshape(-1)

con_dict = {
    "type": "eq",
    "fun": constraints,
}

res = minimize(
    objective,
    z0,
    method="SLSQP",
    constraints=[con_dict],
    options={"maxiter": 200, "ftol": 1e-6},
)

X_opt, U_opt = unpack(res.x)
print("Optimization success:", res.success)
print("Terminal state:", X_opt[-1])
print("First few controls:", U_opt[:5, 0])
      
