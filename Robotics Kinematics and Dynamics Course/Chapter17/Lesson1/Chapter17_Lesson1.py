import numpy as np

# Example dimensions: floating base (6) + n joints
n_joints = 12
ndof = 6 + n_joints

# Configuration vector q = [q_B; q_J]
q = np.zeros(ndof)

# Generalized velocity v = [V_B; qdot_J]
v = np.zeros(ndof)

# Example: set some nonzero base twist and joint velocities
# V_B = [omega_x, omega_y, omega_z, v_x, v_y, v_z]
v[0:6] = np.array([0.0, 0.0, 0.5, 0.1, 0.0, 0.0])   # base twist
v[6:]  = 0.2 * np.ones(n_joints)                   # joint speeds

# Placeholder inertia matrix H(q); in practice use a library call:
#   H = pinocchio.crba(model, data, q)
H = np.eye(ndof)  # for demonstration, identity inertia

# Generalized momentum p = H(q) v
p = H.dot(v)

print("Generalized velocity v:", v)
print("Generalized momentum p:", p)
      
