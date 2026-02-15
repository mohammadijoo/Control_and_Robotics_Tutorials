import numpy as np
import pinocchio as pin

# Load model from URDF
model = pin.buildModelFromUrdf("humanoid.urdf")
data = model.createData()

# Example configuration and velocity (floating base + joints)
q = pin.neutral(model)   # nominal configuration
v = np.zeros(model.nv)   # zero generalized velocity

# Compute centroidal momentum and centroidal momentum matrix
pin.computeCentroidalMomentum(model, data, q, v)
# data.hg is 6D centroidal momentum (linear; angular) about COM
hG = data.hg.copy()

# Compute centroidal momentum matrix A(q)
pin.computeCentroidalMap(model, data, q)
A = data.Ag.copy()  # A has shape (6, nv) so that hG = A @ v

print("Centroidal momentum h_G:", hG)
print("Centroidal momentum matrix A(q):\n", A)
      
