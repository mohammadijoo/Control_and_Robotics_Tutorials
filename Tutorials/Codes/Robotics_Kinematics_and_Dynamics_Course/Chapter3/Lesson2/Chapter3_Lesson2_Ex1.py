from spatialmath import SO3

omega = [0.0, 0.0, 1.0]
theta = np.pi / 2.0

R = SO3.AngleAxis(theta, omega)        # axis-angle to rotation matrix
omega2, theta2 = R.angvec()            # rotation matrix back to axis-angle
      
