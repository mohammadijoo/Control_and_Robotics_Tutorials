
import numpy as np

def dyn_consistent_pinv(M, Jc):
    """
    Compute dynamically consistent pseudo-inverse of Jc:
        Jc_dyn_pinv = M^{-1} Jc^T (Jc M^{-1} Jc^T)^{-1}
    M  : (n,n) positive-definite mass matrix
    Jc : (nc,n) contact Jacobian
    """
    Minv = np.linalg.inv(M)
    # Lambda_c = (Jc M^{-1} Jc^T)^{-1} (operational contact inertia)
    Lambda_c = np.linalg.inv(Jc @ Minv @ Jc.T)
    Jc_dyn_pinv = Minv @ Jc.T @ Lambda_c
    return Jc_dyn_pinv

def contact_projector(M, Jc):
    """
    Nc = I - Jc_dyn_pinv Jc
    """
    n = M.shape[0]
    Jc_dyn_pinv = dyn_consistent_pinv(M, Jc)
    Nc = np.eye(n) - Jc_dyn_pinv @ Jc
    return Nc, Jc_dyn_pinv

def contact_consistent_acc(M, Jc, Jc_dot_qdot, qdd_des):
    """
    Compute contact-consistent acceleration:
        qdd_star = Nc qdd_des - Jc_dyn_pinv Jc_dot_qdot
    Inputs:
      M            : (n,n) mass matrix at current q
      Jc           : (nc,n) contact Jacobian
      Jc_dot_qdot  : (nc,) = Jc_dot(q,qdot) @ qdot (known from dynamics)
      qdd_des      : (n,) unconstrained desired acceleration
    """
    Nc, Jc_dyn_pinv = contact_projector(M, Jc)
    return Nc @ qdd_des - Jc_dyn_pinv @ Jc_dot_qdot

# Example usage (with dummy numbers; in practice, query M, Jc, Jc_dot_qdot from a robotics lib)
n = 6
nc = 3
M = np.eye(n)  # replace with pinocchio.crba(model, data, q)
Jc = np.random.randn(nc, n)  # replace with contact Jacobian from forward kinematics
Jc_dot_qdot = np.zeros(nc)   # replace with pinocchio.computeJdotQdot(...)
qdd_des = np.zeros(n)

qdd_star = contact_consistent_acc(M, Jc, Jc_dot_qdot, qdd_des)
print("Contact-consistent qdd_star:", qdd_star)
