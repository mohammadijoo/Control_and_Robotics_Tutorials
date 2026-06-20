import numpy as np
import control as ctrl

A = np.array([
    [0.0, 1.0, 0.0],
    [0.0, -b / J, K_t / J],
    [0.0, -K_e / L_a, -R_a / L_a]
])
B = np.array([[0.0], [0.0], [1.0 / L_a]])
C = np.array([[1.0, 0.0, 0.0]])  # output: theta
D = np.array([[0.0]])

motor_ss = ctrl.ss(A, B, C, D)

# Time response to a step voltage can be computed by
# t, y = ctrl.step_response(motor_ss)
