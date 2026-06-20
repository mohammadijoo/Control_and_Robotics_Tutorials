import numpy as np

def skew(v):
    """Return 3x3 skew-symmetric matrix S(v) such that S(v) @ x = v x x."""
    v = np.asarray(v).reshape(3,)
    return np.array([[0.0,    -v[2],  v[1]],
                     [v[2],   0.0,   -v[0]],
                     [-v[1],  v[0],  0.0]])

def spatial_inertia(m, c, Ic):
    """
    Construct 6x6 spatial inertia Is at reference point O, given:
      m  : scalar mass
      c  : 3-vector from O to center of mass C
      Ic : 3x3 rotational inertia about C, expressed in frame O's axes
    """
    c = np.asarray(c).reshape(3,)
    Ic = np.asarray(Ic).reshape(3, 3)

    S = skew(c)
    I3 = np.eye(3)

    # Top-left block: Ic - m * S @ S  (equivalently: Ic + m * S @ S.T)
    I11 = Ic - m * S @ S
    I12 = m * S
    I21 = -m * S
    I22 = m * I3

    top = np.hstack((I11, I12))
    bottom = np.hstack((I21, I22))
    return np.vstack((top, bottom))

def kinetic_energy(Is, v):
    """
    Compute kinetic energy T = 0.5 * v^T * Is * v
    where v is a 6-vector spatial velocity at O.
    """
    v = np.asarray(v).reshape(6,)
    return 0.5 * float(v.T @ Is @ v)

def momentum(Is, v):
    """
    Spatial momentum h = Is * v.
    """
    v = np.asarray(v).reshape(6,)
    return Is @ v

# Example: simple link
m = 5.0  # kg
c = np.array([0.0, 0.0, 0.2])  # CoM 20 cm along z from joint frame
Ic = np.diag([0.1, 0.2, 0.15])  # inertia about CoM (kg m^2)

Is = spatial_inertia(m, c, Ic)
print("Spatial inertia Is:\n", Is)

# A sample spatial velocity: omega = [0, 0, 1], v_O = [0.1, 0, 0]
v = np.array([0.0, 0.0, 1.0, 0.1, 0.0, 0.0])
h = momentum(Is, v)
T = kinetic_energy(Is, v)

print("Spatial momentum h:", h)
print("Kinetic energy T:", T)
      
