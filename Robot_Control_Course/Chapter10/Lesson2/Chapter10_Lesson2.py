
import numpy as np

# Dimensions
nx = 4   # state dimension
nu = 2   # input dimension
ny = 2   # output dimension

# Example linearized model (replace with your robot's linearization)
A = np.eye(nx) + 0.01 * np.random.randn(nx, nx)
B = 0.01 * np.random.randn(nx, nu)
C = np.hstack((np.eye(ny), np.zeros((ny, nx - ny))))

N = 10  # horizon length

# Weights
Qy = np.eye(ny)
R = 0.01 * np.eye(nu)

# Build block-diagonal Q and R_blk
Q_blk = np.kron(np.eye(N), Qy)
R_blk = np.kron(np.eye(N), R)

def build_prediction_matrices(A, B, N):
    nx, nu = A.shape[0], B.shape[1]
    Sx = np.zeros((N * nx, nx))
    Su = np.zeros((N * nx, N * nu))

    A_power = np.eye(nx)
    for i in range(N):
        A_power = A_power @ A  # A^(i+1)
        Sx[i*nx:(i+1)*nx, :] = A_power

        for j in range(i+1):
            A_pow = np.linalg.matrix_power(A, i - j)
            Su[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = A_pow @ B
    return Sx, Su

Sx, Su = build_prediction_matrices(A, B, N)
C_blk = np.kron(np.eye(N), C)

T = C_blk @ Su  # maps U to stacked output error

def mpc_step(xk, y_ref_seq):
    """
    xk: current state (nx,)
    y_ref_seq: stacked reference outputs over horizon (N * ny,)
    """
    # Build b = C_blk Sx xk - Y_ref
    b = C_blk @ (Sx @ xk) - y_ref_seq

    H = 2 * (T.T @ Q_blk @ T + R_blk)
    f = 2 * (T.T @ Q_blk @ b)

    # Unconstrained optimum: U* = -H^{-1} f
    U_star = -np.linalg.solve(H, f)

    u0_star = U_star[:nu]
    return u0_star, U_star

# Example usage
xk = np.zeros(nx)
y_ref_seq = np.zeros(N * ny)  # track zero output
u0_star, U_star = mpc_step(xk, y_ref_seq)
print("MPC control at time k:", u0_star)
