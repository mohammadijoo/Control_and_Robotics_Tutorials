import numpy as np
from enum import Enum, auto
from dataclasses import dataclass
from typing import List

class JointType(Enum):
    REVOLUTE = auto()
    PRISMATIC = auto()
    FIXED = auto()

def rot_z(theta: float) -> np.ndarray:
    """Rotation about z-axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])

def homog(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Build 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

@dataclass
class Joint:
    jtype: JointType
    axis: np.ndarray        # 3D unit vector
    offset: float = 0.0     # constant offset (angle or length)

@dataclass
class Link:
    name: str
    length: float
    joint: Joint

@dataclass
class SerialChain:
    links: List[Link]

    def forward_kinematics(self, q: np.ndarray) -> List[np.ndarray]:
        """
        Compute link frames T_i(q) (i = 0..n) in a simple planar-style model:
        all joints rotate about z, link frames lie in xy-plane.
        """
        assert q.shape[0] == len(self.links)
        Ts = []
        T = np.eye(4)  # base frame
        Ts.append(T.copy())
        for i, (link, qi) in enumerate(zip(self.links, q), start=1):
            if link.joint.jtype == JointType.REVOLUTE:
                theta = link.joint.offset + qi
                R = rot_z(theta)
                # translation along x-axis by link.length
                p = np.array([link.length, 0.0, 0.0])
                A_i = homog(R, p)
            elif link.joint.jtype == JointType.PRISMATIC:
                d = link.joint.offset + qi
                R = np.eye(3)
                p = np.array([d, 0.0, 0.0])
                A_i = homog(R, p)
            else:  # FIXED
                A_i = np.eye(4)

            T = T @ A_i
            Ts.append(T.copy())
        return Ts

# Example: 2R planar arm
link1 = Link("L1", length=1.0, joint=Joint(JointType.REVOLUTE, axis=np.array([0, 0, 1])))
link2 = Link("L2", length=0.7, joint=Joint(JointType.REVOLUTE, axis=np.array([0, 0, 1])))
arm = SerialChain([link1, link2])

q = np.array([np.deg2rad(45.0), np.deg2rad(-30.0)])
Ts = arm.forward_kinematics(q)
print("End-effector transform:\n", Ts[-1])
      
