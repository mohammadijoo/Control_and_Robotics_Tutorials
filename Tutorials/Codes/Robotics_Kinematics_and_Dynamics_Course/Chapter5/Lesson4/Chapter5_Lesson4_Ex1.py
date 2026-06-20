# Example with roboticstoolbox (Peter Corke)
import roboticstoolbox as rtb
import numpy as np

# Here we load an existing redundant arm model if available (e.g. KUKA LBR iiwa 7-DOF)
# For illustration, we assume such a model exists in the toolbox.
robot = rtb.models.DH.Puma560()  # nonredundant example
# In practice, use a 7-DOF model such as robot = rtb.models.DH.KUKAiiwa() if provided.

q = np.array([0, 0.3, -0.5, 0.4, -0.2, 0.1])
T = robot.fkine(q)  # returns SE3 object
print(T.A)  # 4x4 homogeneous transform
      
