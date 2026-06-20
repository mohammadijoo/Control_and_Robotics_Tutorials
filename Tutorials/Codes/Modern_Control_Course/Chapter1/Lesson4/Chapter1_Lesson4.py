import numpy as np
import control as ct  # pip install control

# State, input, output dimensions: n = 2, m = 2, p = 2
A = np.array([[0.0, 1.0],
              [-4.0, -2.0]])
B = np.array([[1.0, 0.0],
              [0.0, 1.0]])
C = np.eye(2)          # outputs equal states
D = np.zeros((2, 2))

# Build continuous-time state-space system
sys = ct.ss(A, B, C, D)

# Step response where both inputs step from 0 to 1
T, yout = ct.step_response(sys)

print("Time vector shape:", T.shape)
print("Output array shape (p, m, len(T)):", yout.shape)

# Examine response of y1 and y2 to a step in u1 only
T1, yout1 = ct.step_response(sys, input=0)  # index 0 for u1
print("Step response from u1 to y1, y2 at final time:", yout1[:, -1])
      
