import numpy as np
from dataclasses import dataclass

@dataclass
class LinkInertialURDF:
    mass: float               # m > 0
    com: np.ndarray           # shape (3,), CoM in link frame
    I_origin: np.ndarray      # shape (3,3), inertia about link origin

def skew(v: np.ndarray) -> np.ndarray:
    """Return the 3x3 skew-symmetric matrix [v]_x."""
    vx, vy, vz = v
    return np.array([
        [0.0, -vz,  vy],
        [vz,  0.0, -vx],
        [-vy, vx,  0.0],
    ])

def spatial_inertia_from_urdf(link: LinkInertialURDF) -> np.ndarray:
    """
    Build 6x6 spatial inertia matrix from URDF-like inertial data.
    Assumes:
      - com is expressed in the same frame as I_origin.
      - I_origin is inertia about link origin, expressed in link frame.
    """
    m = link.mass
    c = link.com.reshape(3, 1)
    I_O = link.I_origin

    S = skew(c.flatten())
    upper_left = I_O + m * S @ S.T
    upper_right = m * S
    lower_left = -m * S
    lower_right = m * np.eye(3)

    I_spatial = np.block([
        [upper_left,  upper_right],
        [lower_left,  lower_right],
    ])
    return I_spatial

def inertial_parameters_10d(link: LinkInertialURDF) -> np.ndarray:
    """Return 10-dim inertial parameter vector [m, mc, I_components]^T."""
    m = link.mass
    cx, cy, cz = link.com
    I = link.I_origin
    pi = np.array([
        m,
        m * cx, m * cy, m * cz,
        I[0, 0], I[1, 1], I[2, 2],
        I[0, 1], I[1, 2], I[0, 2],
    ])
    return pi

# Example: simple numeric test
link = LinkInertialURDF(
    mass=5.0,
    com=np.array([0.1, 0.0, 0.0]),
    I_origin=np.array([
        [0.2, 0.0, 0.0],
        [0.0, 0.3, 0.0],
        [0.0, 0.0, 0.4],
    ])
)

I_spatial = spatial_inertia_from_urdf(link)
pi_10 = inertial_parameters_10d(link)

print("Spatial inertia:\n", I_spatial)
print("10D inertial parameters:\n", pi_10)

# Simple symmetry and positive-definiteness checks
sym_err = np.linalg.norm(I_spatial - I_spatial.T)
eigvals = np.linalg.eigvalsh(I_spatial)
print("Symmetry error:", sym_err)
print("Eigenvalues:", eigvals)
      
