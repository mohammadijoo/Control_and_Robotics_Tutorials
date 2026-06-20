import numpy as np
from scipy.linalg import expm
from scipy.integrate import solve_ivp

def fundamental_matrix_LTI(A, t0, t):
    """
    Phi(t) for constant A with normalization Phi(t0) = I:
      Phi(t) = expm(A*(t-t0))
    """
    return expm(A * (t - t0))

def fundamental_matrix_LTV(Afun, t0, t_eval):
    """
    Compute Phi(t) for time-varying A(t) by integrating:
      Phi_dot = A(t) Phi,  Phi(t0)=I
    Vectorize Phi into a length n*n vector to use solve_ivp.
    """
    t_eval = np.asarray(t_eval)
    n = Afun(t0).shape[0]
    Phi0 = np.eye(n).reshape(-1)

    def ode(t, phi_vec):
        Phi = phi_vec.reshape(n, n)
        dPhi = Afun(t) @ Phi
        return dPhi.reshape(-1)

    sol = solve_ivp(ode, (t0, float(t_eval[-1])), Phi0, t_eval=t_eval, rtol=1e-9, atol=1e-12)
    Phi_list = [sol.y[:, k].reshape(n, n) for k in range(sol.y.shape[1])]
    return Phi_list

# Example 1 (LTI): x_dot = A x
A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
t0 = 0.0
x0 = np.array([1.0, 0.0])

t = 1.25
Phi = fundamental_matrix_LTI(A, t0, t)
x_t = Phi @ x0
print("Phi(t):\n", Phi)
print("x(t): ", x_t)

# Example 2 (LTV): A(t) = [[0, 1], [-(2+0.1 t), -3]]
def Afun(t):
    return np.array([[0.0, 1.0],
                     [-(2.0 + 0.1*t), -3.0]])

t_grid = np.linspace(0.0, 2.0, 21)
Phi_grid = fundamental_matrix_LTV(Afun, 0.0, t_grid)

# Solve IVP x(t) = Phi(t) x0 when Phi(0)=I
x_grid = np.array([Phi_grid[k] @ x0 for k in range(len(t_grid))])
print("x(2.0) approx:", x_grid[-1])
