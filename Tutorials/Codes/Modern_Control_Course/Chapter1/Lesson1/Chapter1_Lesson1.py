import numpy as np
from control import ss, step_response    # python-control library

# Physical parameters
m = 1.0
c = 0.4
k = 2.0

# State-space matrices for the mass-spring-damper
A = np.array([[0.0,        1.0],
              [-k/m,  -c/m]])
B = np.array([[0.0],
              [1.0/m]])
C = np.array([[1.0, 0.0]])   # measure position
D = np.array([[0.0]])

system = ss(A, B, C, D)

# Compute step response of position to a unit step in force
T, y = step_response(system)

# T contains time samples, y contains output samples
print("First 5 time samples:", T[:5])
print("First 5 output samples:", y[0][:5])
      
