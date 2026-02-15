import numpy as np

class TactileContact:
    def __init__(self, positions, pressures, areas,
                 c_des, n_des, mu):
        """
        positions: (K,3) array of taxel positions p_{j,k}
        pressures: (K,) array of pressures sigma_{j,k}
        areas:     (K,) array of taxel areas a_k
        c_des:     (3,) desired contact position
        n_des:     (3,) desired surface normal (unit)
        mu:        scalar friction coefficient
        """
        self.positions = np.asarray(positions, dtype=float)
        self.pressures = np.asarray(pressures, dtype=float)
        self.areas = np.asarray(areas, dtype=float)
        self.c_des = np.asarray(c_des, dtype=float)
        self.n_des = np.asarray(n_des, dtype=float)
        self.mu = float(mu)

def center_of_pressure(contact):
    w = contact.pressures * contact.areas
    F_n = np.sum(w) + 1e-9  # avoid division by zero
    c = (w[:, None] * contact.positions).sum(axis=0) / F_n
    return F_n, c

def slip_indicator(F_n, f_t_norm, mu):
    # h_j = ||f_t|| / (mu F_n) - 1
    return f_t_norm / (mu * F_n + 1e-9) - 1.0

def tactile_grasp_refine_step(q, contacts, Jc_fn, Jn_fn,
                              w_slip=1.0, w_pos=1.0, w_align=0.1,
                              step_size=1e-2,
                              q_min=None, q_max=None):
    """
    q:        (n,) current joint configuration
    contacts: list of TactileContact objects, one per fingertip
    Jc_fn:    function (q, j) -> (3,n) Jacobian of contact position
    Jn_fn:    function (q, j) -> (3,n) Jacobian of contact normal
    """
    q = np.asarray(q, dtype=float)
    n = q.size
    grad = np.zeros_like(q)

    for j, contact in enumerate(contacts):
        F_n_est, c_est = center_of_pressure(contact)

        # For illustration we use a crude proxy for tangential force norm
        # as the magnitude of deviation from the desired contact location.
        pos_err = c_est - contact.c_des
        f_t_norm_est = np.linalg.norm(pos_err)

        h = slip_indicator(F_n_est, f_t_norm_est, contact.mu)
        slip_penalty = max(0.0, h)

        Jc = Jc_fn(q, j)  # shape (3,n)
        Jn = Jn_fn(q, j)  # shape (3,n)

        # Position gradient: d/dq ||c(q) - c_des||^2 = 2 Jc^T (c - c_des)
        grad_pos = 2.0 * Jc.T @ pos_err

        # Normal alignment (assuming we can estimate surface normal from kinematics)
        n_est = np.zeros(3,)  # placeholder; use geometric model
        n_err = n_est - contact.n_des
        grad_align = 2.0 * Jn.T @ n_err

        # Slip penalty gradient (simple heuristic)
        # d/dq max(0, h(q))^2 ≈ 2 max(0,h) * (dh/dq)
        # Here we approximate dh/dq via Jc and normalize.
        if slip_penalty > 0.0:
            if np.linalg.norm(pos_err) > 1e-6:
                dir_vec = pos_err / np.linalg.norm(pos_err)
                dh_dq = (Jc.T @ dir_vec) / (contact.mu * F_n_est + 1e-9)
            else:
                dh_dq = np.zeros_like(q)
            grad_slip = 2.0 * slip_penalty * dh_dq
        else:
            grad_slip = np.zeros_like(q)

        grad += (w_pos * grad_pos
                 + w_align * grad_align
                 + w_slip * grad_slip)

    # Gradient descent step
    dq = -step_size * grad
    q_new = q + dq

    # Project onto joint limits if provided
    if q_min is not None:
        q_new = np.maximum(q_new, np.asarray(q_min))
    if q_max is not None:
        q_new = np.minimum(q_new, np.asarray(q_max))

    return q_new
      
