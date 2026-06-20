import numpy as np

# Example from Section 5
A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0, 0.0],
              [4.0, -1.0]])
C = np.array([[1.0, 0.0],
              [0.0, 1.0]])
D = np.array([[0.0, 0.0],
              [0.0, 0.5]])

# Build MIMO state-space model
import control as ctrl
sys = ctrl.ss(A, B, C, D)

# Time grid
t = np.linspace(0.0, 10.0, 2001)

# Inputs: u1 is a step, u2 is a decaying exponential
u1 = np.ones_like(t)
u2 = np.exp(-0.7 * t)

# Stack inputs as shape (m, len(t)) expected by python-control
U = np.vstack([u1, u2])

# Simulate with zero initial state
t_out, y_out, x_out = ctrl.forced_response(sys, T=t, U=U, X0=np.zeros(2), return_x=True)

print("y_out shape:", y_out.shape)  # (p, N)
print("x_out shape:", x_out.shape)  # (n, N)

# Optional: verify that y2 = x2 + 0.5 u2 numerically
y2_check = x_out[1, :] + 0.5 * u2
max_err = np.max(np.abs(y_out[1, :] - y2_check))
print("max|y2 - (x2+0.5u2)| =", max_err)
      
