
import numpy as np

# Planar 2-DOF forward kinematics and Jacobian
def fk_planar_2dof(q, l1=1.0, l2=1.0):
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y])

def jac_planar_2dof(q, l1=1.0, l2=1.0):
    q1, q2 = q
    J = np.zeros((2, 2))
    J[0, 0] = -l1 * np.sin(q1) - l2 * np.sin(q1 + q2)
    J[0, 1] = -l2 * np.sin(q1 + q2)
    J[1, 0] =  l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    J[1, 1] =  l2 * np.cos(q1 + q2)
    return J

# Placeholder for an inverse-dynamics function from previous lessons
def inverse_dynamics(q, qd, qdd_des):
    """
    Map desired joint accelerations qdd_des to torques tau using
    the known robot dynamics M(q), C(q, qd), g(q).
    Here we just return qdd_des as a stand-in.
    """
    return qdd_des

def joint_space_pd(q, qd, q_des, qd_des, Kp, Kd):
    e_q  = q_des - q
    e_qd = qd_des - qd
    qdd_des = Kp @ e_q + Kd @ e_qd
    tau = inverse_dynamics(q, qd, qdd_des)
    return tau

def task_space_pd(q, qd, x_des, xd_des, Kx, D_x, l1=1.0, l2=1.0):
    # Current task state
    x  = fk_planar_2dof(q, l1, l2)
    J  = jac_planar_2dof(q, l1, l2)
    # Task-space velocity
    xdot = J @ qd

    # Task-space errors
    e_x  = x_des  - x
    e_xd = xd_des - xdot

    # Desired task-space acceleration (simple PD)
    xdd_des = Kx @ e_x + D_x @ e_xd

    # Map task acceleration to joint acceleration (pseudoinverse)
    J_pinv = np.linalg.pinv(J)
    qdd_des = J_pinv @ xdd_des

    tau = inverse_dynamics(q, qd, qdd_des)
    return tau

if __name__ == "__main__":
    # Example usage
    q  = np.array([0.4, 0.2])
    qd = np.array([0.0, 0.0])
    q_des  = np.array([0.5, 0.3])
    qd_des = np.array([0.0, 0.0])

    x_des  = fk_planar_2dof(q_des)  # consistent with q_des
    xd_des = np.array([0.0, 0.0])

    Kp = np.diag([30.0, 30.0])
    Kd = np.diag([10.0, 10.0])
    Kx = np.diag([50.0, 50.0])
    Dx = np.diag([20.0, 20.0])

    tau_joint = joint_space_pd(q, qd, q_des, qd_des, Kp, Kd)
    tau_task  = task_space_pd(q, qd, x_des, xd_des, Kx, Dx)

    print("Tau (joint-space goal) =", tau_joint)
    print("Tau (task-space goal)  =", tau_task)
