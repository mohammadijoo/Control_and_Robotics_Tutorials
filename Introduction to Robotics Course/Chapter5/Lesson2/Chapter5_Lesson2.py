
from dataclasses import dataclass
from typing import List

@dataclass
class Joint:
    name: str
    fi: int  # DOF allowed by joint

def mobility_spatial(L: int, joints: List[Joint]) -> int:
    J = len(joints)
    sum_f = sum(j.fi for j in joints)
    M = 6 * (L - 1 - J) + sum_f
    return M

# Example 1: serial chain with 6 revolute joints
serial_joints = [Joint(f"R{i+1}", 1) for i in range(6)]
print("Serial M =", mobility_spatial(L=7, joints=serial_joints))

# Example 2: planar 5-bar (approx spatial count not ideal, but illustrative)
# Suppose L=5 links incl. base, J=5 revolute joints
fivebar_joints = [Joint(f"R{i+1}", 1) for i in range(5)]
print("Five-bar M (spatial formula) =", mobility_spatial(L=5, joints=fivebar_joints))
      