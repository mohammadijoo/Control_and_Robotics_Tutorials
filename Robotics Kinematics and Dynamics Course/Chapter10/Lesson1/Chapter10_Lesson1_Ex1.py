# Requires: pip install roboticstoolbox-python spatialmath
import numpy as np
import roboticstoolbox as rtb

# Planar 2R model
arm = rtb.models.DH.Planar2()

q = np.array([0.5, -0.3])
qd = np.array([0.4, 0.2])

M = arm.inertia(q)        # inertia matrix M(q)
g_vec = arm.gravload(q)   # gravity torques g(q) (related to V(q))

T = 0.5 * float(qd.T @ M @ qd)

print("M(q) =\n", M)
print("Kinetic energy T =", T)
      
