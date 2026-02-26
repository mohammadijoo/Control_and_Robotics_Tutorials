import pybullet as p
import pybullet_data
import time
import numpy as np

def setup_sim():
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    plane_id = p.loadURDF("plane.urdf")
    return physics_client, plane_id

def load_scene_box():
    """Load a box as a URDF; use your own URDF or generated mesh."""
    box_id = p.loadURDF("cube_small.urdf", [0, 0, 0.02])
    return box_id

def load_gripper():
    # Replace with your gripper URDF (parallel-jaw)
    gripper_id = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0])
    return gripper_id

def attempt_grasp(gripper_id, box_id, grasp_pose_world, close_joints, lift_height=0.15, steps=240):
    """
    grasp_pose_world: (pos, orn) of gripper frame in world coordinates.
    close_joints: list of joint positions for grasp closure.
    Returns True if object lifted and held.
    """
    pos, orn = grasp_pose_world
    # Teleport end-effector near grasp pose (for simplicity)
    # In practice, solve IK instead.
    ee_link = 11  # e.g. Franka Panda link index
    p.resetBasePositionAndOrientation(gripper_id, [0, 0, 0], [0, 0, 0, 1])
    # Here you would compute joint positions via IK solver

    # Close fingers
    for j, q in close_joints:
        p.setJointMotorControl2(gripper_id, j,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=q, force=50.0)

    for _ in range(steps):
        p.stepSimulation()

    # Lift motion (again, simplified as base motion or EE joint update)
    box_pos, _ = p.getBasePositionAndOrientation(box_id)
    start_z = box_pos[2]
    for _ in range(steps):
        # here move EE upwards using joint targets
        p.stepSimulation()

    box_pos, _ = p.getBasePositionAndOrientation(box_id)
    lifted = box_pos[2] > start_z + 0.05
    return lifted

def evaluate_grasps_in_sim(candidates, K=10):
    physics_client, plane_id = setup_sim()
    box_id = load_scene_box()
    gripper_id = load_gripper()

    # Suppose candidates is list of (grasp_pose_world, quality)
    topK = sorted(candidates, key=lambda x: x[1], reverse=True)[:K]
    results = []
    for grasp_pose_world, q in topK:
        success = attempt_grasp(gripper_id, box_id, grasp_pose_world,
                                close_joints=[(9, 0.04), (10, 0.04)])
        results.append((grasp_pose_world, q, success))
    p.disconnect(physics_client)
    return results
      
