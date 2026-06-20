import numpy as np

# Unscaled companion-form matrices
A = np.array([[0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0],
              [-1.0, -10.0, -1000.0]])
B = np.array([[0.0],
              [0.0],
              [1.0]])
C = np.array([[1.0, 0.0, 0.0]])
D = np.array([[0.0]])

# Choose a frequency scaling omega and build S = diag(1, omega, omega^2)
omega = 10.0
S = np.diag([1.0, omega, omega**2])
Sinv = np.linalg.inv(S)

# Scaled realization
A_s = Sinv @ A @ S
B_s = Sinv @ B
C_s = C @ S
D_s = D.copy()

print("A_s =\n", A_s)
print("B_s =\n", B_s)
print("C_s =\n", C_s)

# Simulate both systems with the same physical initial condition x0 and input u(t)
from scipy.integrate import solve_ivp

def u_of_t(t):
    # unit step input (piecewise constant)
    return 1.0

def f_unscaled(t, x):
    return (A @ x.reshape(-1,1) + B * u_of_t(t)).flatten()

def f_scaled(t, z):
    return (A_s @ z.reshape(-1,1) + B_s * u_of_t(t)).flatten()

t0, tf = 0.0, 0.02
t_eval = np.linspace(t0, tf, 400)

x0 = np.array([0.0, 0.0, 0.0])              # physical initial state
z0 = (Sinv @ x0.reshape(-1,1)).flatten()    # consistent scaled initial state

sol_x = solve_ivp(f_unscaled, (t0, tf), x0, t_eval=t_eval, rtol=1e-9, atol=1e-12)
sol_z = solve_ivp(f_scaled,   (t0, tf), z0, t_eval=t_eval, rtol=1e-9, atol=1e-12)

# Recover x from z: x = S z
x_from_z = (S @ sol_z.y).T
y_unscaled = (C @ sol_x.y).flatten()
y_scaled   = (C @ x_from_z.T).flatten()

# The two outputs should match (up to solver tolerances)
max_err = np.max(np.abs(y_unscaled - y_scaled))
print("max |y_unscaled - y_scaled| =", max_err)

# Optional: use python-control for LTI object handling
try:
    import control
    sys_unscaled = control.ss(A, B, C, D)
    sys_scaled   = control.ss(A_s, B_s, C_s, D_s)
    # Note: internal states differ; outputs coincide when mapped consistently.
    print("Created control.ss objects successfully.")
except Exception as e:
    print("python-control not available in this environment:", e)
