import numpy as np

def euler_zyx_to_R(phi, theta, psi):
    """
    ZYX (yaw-pitch-roll) Euler angles to rotation matrix.
    Angles in radians.
    """
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth,  sth  = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    R = np.array([
        [cpsi * cth,  cpsi * sth * sphi - spsi * cphi,  cpsi * sth * cphi + spsi * sphi],
        [spsi * cth,  spsi * sth * sphi + cpsi * cphi,  spsi * sth * cphi - cpsi * sphi],
        [-sth,        cth * sphi,                      cth * cphi]
    ])
    return R

def R_to_euler_zyx(R):
    """
    Inverse mapping R -> (phi, theta, psi) for ZYX convention.
    Assumes R is a valid rotation matrix.
    """
    # Protect against numerical issues by clipping
    sth = -R[2, 0]
    sth = np.clip(sth, -1.0, 1.0)
    theta = np.arcsin(sth)

    cth = np.cos(theta)
    if abs(cth) > 1e-6:
        phi = np.arctan2(R[2, 1], R[2, 2])
        psi = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock: theta ~ +/- pi/2
        # psi and phi not uniquely defined; choose one convention
        phi = 0.0
        psi = np.arctan2(-R[0, 1], R[1, 1])

    return phi, theta, psi

def euler_zyx_to_omega(phi, theta, psi, phi_dot, theta_dot, psi_dot):
    """
    Compute body angular velocity omega given Euler angles and their rates.
    """
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth,  sth  = np.cos(theta), np.sin(theta)

    T = np.array([
        [1.0, 0.0, -sth],
        [0.0, cphi, cth * sphi],
        [0.0, -sphi, cth * cphi]
    ])
    qdot = np.array([phi_dot, theta_dot, psi_dot])
    return T @ qdot

# Example usage
if __name__ == "__main__":
    phi, theta, psi = 0.2, -0.3, 0.4
    R = euler_zyx_to_R(phi, theta, psi)
    phi_rec, theta_rec, psi_rec = R_to_euler_zyx(R)
    print("R =\n", R)
    print("Recovered angles:", phi_rec, theta_rec, psi_rec)
      
