import numpy as np
import modern_robotics as mr

def fk_6r_poe(q, Slist, M):
    """
    PoE FK for a 6R manipulator.
    Slist: 6x6 matrix whose columns are the screw axes S_i in the space frame.
    M: 4x4 home configuration of the end-effector.
    q: iterable of six joint angles.
    """
    q = np.asarray(q).flatten()
    return mr.FKinSpace(M, Slist, q)

# Example skeleton: the actual Slist depends on robot geometry.
# Here we just show the shape and usage.
Slist_example = np.zeros((6, 6))  # Replace with real screw axes
M_example = np.eye(4)             # Replace with real home pose
q_example = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

T_example = fk_6r_poe(q_example, Slist_example, M_example)
print("T0_6(q_example) =\n", T_example)
      
