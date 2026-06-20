# Chapter26_Lesson5.py
# Servo design by state augmentation for a mass-spring-damper plant.
# Libraries: NumPy, SciPy, Matplotlib. Optional python-control can be used similarly.

import numpy as np
from scipy.signal import place_poles
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    Ak = np.eye(n)
    for _ in range(1, n):
        Ak = Ak @ A
        blocks.append(Ak @ B)
    return np.hstack(blocks)


# Plant: x1 = position, x2 = velocity, u = force, y = position.
A = np.array([[0.0, 1.0],
              [-2.0, -0.8]])
B = np.array([[0.0],
              [1.0]])
C = np.array([[1.0, 0.0]])
D = np.array([[0.0]])

n = A.shape[0]
p = C.shape[0]
m = B.shape[1]

# Integral state: eta_dot = r - y = r - Cx for D = 0.
Aa = np.block([
    [A, np.zeros((n, p))],
    [-C, np.zeros((p, p))]
])
Ba = np.vstack([B, np.zeros((p, m))])
Ea = np.vstack([np.zeros((n, p)), np.eye(p)])

Wc = controllability_matrix(Aa, Ba)
rank_Wc = np.linalg.matrix_rank(Wc)
print("Augmented controllability rank:", rank_Wc, "of", n + p)
if rank_Wc < n + p:
    raise RuntimeError("The augmented plant is not controllable for this integral servo design.")

# Desired augmented closed-loop poles. The integrator adds one extra pole.
desired_poles = np.array([-2.0, -2.5, -3.0])
Kaug = place_poles(Aa, Ba, desired_poles).gain_matrix
Kx = Kaug[:, :n]
Ki = -Kaug[:, n:]

print("Kx =", Kx)
print("Ki =", Ki)
print("Closed-loop poles =", np.linalg.eigvals(Aa - Ba @ Kaug))

Acl = Aa - Ba @ Kaug
Cr = np.hstack([C, np.zeros((p, p))])

r_value = 1.0

def closed_loop_rhs(t, z):
    return (Acl @ z.reshape(-1, 1) + Ea * r_value).ravel()

sol = solve_ivp(closed_loop_rhs, (0.0, 8.0), np.zeros(n + p), max_step=0.01, dense_output=True)
t = np.linspace(0.0, 8.0, 801)
z = sol.sol(t)
y = (Cr @ z).ravel()
eta = z[-1, :]
u = np.array([(-Kx @ z[:n, [i]] + Ki @ z[n:, [i]])[0, 0] for i in range(z.shape[1])])
error = r_value - y

print("Final output y(T) =", y[-1])
print("Final tracking error =", error[-1])
print("Final control input =", u[-1])

plt.figure()
plt.plot(t, y, label="y(t)")
plt.plot(t, r_value * np.ones_like(t), "--", label="r")
plt.xlabel("Time [s]")
plt.ylabel("Output")
plt.title("State-Augmented Servo Response")
plt.grid(True)
plt.legend()

plt.figure()
plt.plot(t, u, label="u(t)")
plt.xlabel("Time [s]")
plt.ylabel("Control input")
plt.title("Servo Control Effort")
plt.grid(True)
plt.legend()
plt.show()
