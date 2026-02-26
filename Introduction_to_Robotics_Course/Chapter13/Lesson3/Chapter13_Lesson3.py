import pybullet as p
import pybullet_data
import time

cid = p.connect(p.GUI)  # or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")
box = p.loadURDF("r2d2.urdf", [0, 0, 0.2])

dt = 1.0/240.0
p.setTimeStep(dt)

for k in range(1000):
    # Example control: zero torques (free fall)
    p.stepSimulation()
    pos, orn = p.getBasePositionAndOrientation(box)
    if k % 120 == 0:
        print("t=", k*dt, "pos=", pos)
    time.sleep(dt)
p.disconnect()
