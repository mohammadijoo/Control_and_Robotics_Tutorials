import pinocchio as pin

# Load model from URDF
model = pin.buildSampleModelHumanoidRandom()
data = model.createData()

# q: configuration vector including floating base (dimension model.nq)
# v: velocity vector including base twist (dimension model.nv)
q = pin.randomConfiguration(model)
v = np.zeros(model.nv)

# Forward kinematics and Jacobians in one call
pin.forwardKinematics(model, data, q, v)
frame_id = model.getFrameId("left_foot")
J6 = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
      
