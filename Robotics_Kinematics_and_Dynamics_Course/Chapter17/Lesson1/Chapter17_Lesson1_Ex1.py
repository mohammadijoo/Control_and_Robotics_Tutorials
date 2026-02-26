import pinocchio as pin

# model, data constructed from a URDF including a floating base
# by specifying a joint model like JointModelFreeFlyer
# model, data = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())

q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)

# Composite rigid-body algorithm to compute H(q)
H = pin.crba(model, data, q)

p = H.dot(v)
      
