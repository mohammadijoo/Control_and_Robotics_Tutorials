import numpy as np

L1, L2 = 1.0, 1.0

def fk_planar_2R(q, l1=L1, l2=L2):
    theta1, theta2 = q
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return np.array([x, y])

def jacobian_fk(q, l1=L1, l2=L2):
    theta1, theta2 = q
    s1 = np.sin(theta1)
    c1 = np.cos(theta1)
    s12 = np.sin(theta1 + theta2)
    c12 = np.cos(theta1 + theta2)
    J = np.array([
        [-l1 * s1 - l2 * s12, -l2 * s12],
        [ l1 * c1 + l2 * c12,  l2 * c12]
    ])
    return J

def constraint(q, p_des):
    return fk_planar_2R(q) - p_des

def project_onto_task_manifold(q_init, p_des, max_iters=20, tol=1e-10):
    q = np.array(q_init, dtype=float)
    for k in range(max_iters):
        h_q = constraint(q, p_des)
        J = jacobian_fk(q)
        # Pseudoinverse of 2x2 Jacobian using SVD
        U, S, Vt = np.linalg.svd(J)
        J_pinv = Vt.T @ np.diag(1.0 / S) @ U.T
        q = q - J_pinv @ h_q
        if np.linalg.norm(h_q) < tol:
            break
    return q

if __name__ == "__main__":
    p_des = np.array([1.0, 1.0])
    q_guess = np.array([0.1, 0.2])
    q_proj = project_onto_task_manifold(q_guess, p_des)
    print("Projected configuration:", q_proj)
    print("End-effector position:", fk_planar_2R(q_proj))
      
