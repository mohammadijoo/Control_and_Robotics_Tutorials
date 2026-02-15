
import numpy as np
import pinocchio as pin
from qpsolvers import solve_qp

# Assume we have a Pinocchio model of a simple humanoid
# with data structure `model`, `data` already created.
# CoM and foot frames:
com_ref = np.array([0.0, 0.0, 0.8])   # desired CoM
swing_ref = np.array([0.1, 0.1, 0.4]) # desired swing foot position
posture_ref = None  # to be set to a nominal configuration q0

Kp_com = 50.0
Kd_com = 10.0
Kp_swing = 150.0
Kd_swing = 20.0
Kp_post = 20.0
Kd_post = 5.0

def build_whole_body_qp(model, data, q, qd, support_frame, swing_frame):
    """
    Build the matrices H, g, Aeq, beq, Aineq, bineq
    for a simple whole-body QP with CoM, swing foot,
    and posture tasks.
    """
    nv = model.nv  # velocity dimension
    na = model.nv  # assume all joints actuated for simplicity

    # 1) Compute dynamics terms
    pin.computeAllTerms(model, data, q, qd)
    M = data.M.copy()
    h = data.nle.copy()   # non-linear effects

    # Selection matrix for actuated joints (identity here)
    S = np.eye(na, nv)

    # 2) CoM quantities
    com_pos = pin.centerOfMass(model, data, q)
    com_vel = data.vcom[0]
    J_com = pin.jacobianCenterOfMass(model, data, q)

    com_pos_err = com_ref - com_pos
    com_vel_err = -com_vel
    com_acc_des = (0.0
                   + Kp_com * com_pos_err
                   + Kd_com * com_vel_err)

    # CoM task: J_com * qdd = com_acc_des - Jdot_com * qd
    # Pinocchio does not give Jdot_com directly; we can approximate with finite difference
    # or use a dedicated function in more complete code.
    Jdot_com_qd = np.zeros(3)  # placeholder for simplicity
    A_com = J_com
    b_com = com_acc_des - Jdot_com_qd

    # 3) Swing foot task
    # Frame placement and Jacobian
    swing_id = model.getFrameId(swing_frame)
    pin.updateFramePlacement(model, data, swing_id)
    x_swing = data.oMf[swing_id].translation
    J_swing = pin.computeFrameJacobian(
        model, data, q, swing_id, pin.LOCAL_WORLD_ALIGNED
    )[:3, :]

    # Approximate velocity and Jdot * qd
    swing_vel = J_swing.dot(qd)
    x_err = swing_ref - x_swing
    v_err = -swing_vel
    acc_des_swing = (0.0
                     + Kp_swing * x_err
                     + Kd_swing * v_err)
    Jdot_swing_qd = np.zeros(3)  # placeholder

    A_swing = J_swing
    b_swing = acc_des_swing - Jdot_swing_qd

    # 4) Posture task
    global posture_ref
    if posture_ref is None:
        posture_ref = q.copy()
    q_err = posture_ref - q
    qd_err = -qd
    acc_des_post = (0.0
                    + Kp_post * q_err
                    + Kd_post * qd_err)
    A_post = np.eye(nv)
    b_post = acc_des_post

    # Stack tasks into least-squares cost:
    w_com = 1.0
    w_swing = 1.0
    w_post = 0.1
    w_reg = 1e-4

    A_tasks = np.vstack([
        np.sqrt(w_com) * A_com,
        np.sqrt(w_swing) * A_swing,
        np.sqrt(w_post) * A_post
    ])
    b_tasks = np.concatenate([
        np.sqrt(w_com) * b_com,
        np.sqrt(w_swing) * b_swing,
        np.sqrt(w_post) * b_post
    ])

    # Cost: 0.5 * ||A_tasks * qdd - b_tasks||^2 + 0.5 * w_reg ||qdd||^2
    H_qdd = A_tasks.T @ A_tasks + w_reg * np.eye(nv)
    g_qdd = -A_tasks.T @ b_tasks

    # Decision vector: z = [qdd, tau] (ignore contact forces for simplicity)
    # Introduce approximate dynamics: M * qdd + h = S.T * tau
    # so tau = (S^-T) * (M * qdd + h); here S is identity.
    # We eliminate tau instead of including it in z.
    H = H_qdd
    g = g_qdd

    # Equality constraints: contact acceleration (stance foot)
    support_id = model.getFrameId(support_frame)
    pin.updateFramePlacement(model, data, support_id)
    Jc = pin.computeFrameJacobian(
        model, data, q, support_id, pin.LOCAL_WORLD_ALIGNED
    )[:3, :]
    Jcdot_qd = np.zeros(3)  # placeholder
    Aeq = Jc
    beq = -Jcdot_qd

    # Inequalities (e.g. joint acceleration bounds)
    qdd_max = 20.0 * np.ones(nv)
    qdd_min = -20.0 * np.ones(nv)
    Aineq = np.vstack([np.eye(nv), -np.eye(nv)])
    bineq = np.concatenate([qdd_max, -qdd_min])

    return H, g, Aeq, beq, Aineq, bineq

def solve_whole_body_qp(H, g, Aeq, beq, Aineq, bineq):
    # qpsolvers interface: solve_qp(P, q, G, h, A, b)
    qdd_star = solve_qp(H, g, Aineq, bineq, Aeq, beq, solver="osqp")
    return qdd_star
