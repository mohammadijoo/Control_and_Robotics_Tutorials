import numpy as np

# ----- Synthetic demonstrations for a linear policy u = theta^T o -----
np.random.seed(0)
N, d = 500, 4
O = np.random.randn(N, d)
theta_star = np.array([1.0, -0.5, 0.3, 0.2])
U = O @ theta_star + 0.05*np.random.randn(N)  # demo actions

# Least-squares imitation learning
theta_hat, *_ = np.linalg.lstsq(O, U, rcond=None)
print("theta_hat =", theta_hat)

# ----- Sim-to-real idea: randomize A,B and test a linear controller -----
A_nom = np.array([[0.9, 0.1],[0.0, 0.85]])
B_nom = np.array([[0.1],[0.2]])
K = np.array([[1.2, 0.4]])  # stable feedback

def rollout(phi_scale=0.1, T=50):
    # randomize parameters
    A = A_nom + phi_scale*np.random.randn(*A_nom.shape)
    B = B_nom + phi_scale*np.random.randn(*B_nom.shape)
    x = np.array([[1.0],[0.0]])
    xs = []
    for t in range(T):
        u = -K @ x               # model-based controller
        x = A @ x + B @ u
        xs.append(x.ravel())
    return np.array(xs)

trajs = [rollout() for _ in range(20)]
print("Final states (randomized sims):", [tr[-1] for tr in trajs[:3]])
      