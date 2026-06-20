import numpy as np

# Robot model: 2R planar arm
class Planar2RModel:
    def __init__(self, L1, L2,
                 q_min=np.array([-np.pi, -np.pi]),
                 q_max=np.array([ np.pi,  np.pi])):
        self.L1 = L1
        self.L2 = L2
        self.q_min = q_min
        self.q_max = q_max

    def fk(self, q):
        q1, q2 = q
        x = self.L1 * np.cos(q1) + self.L2 * np.cos(q1 + q2)
        y = self.L1 * np.sin(q1) + self.L2 * np.sin(q1 + q2)
        return np.array([x, y])

    def jacobian(self, q):
        q1, q2 = q
        s1 = np.sin(q1)
        c1 = np.cos(q1)
        s12 = np.sin(q1 + q2)
        c12 = np.cos(q1 + q2)
        J = np.array([
            [-self.L1 * s1 - self.L2 * s12, -self.L2 * s12],
            [ self.L1 * c1 + self.L2 * c12,  self.L2 * c12],
        ])
        return J

    def clip_to_limits(self, q):
        return np.minimum(np.maximum(q, self.q_min), self.q_max)


def ik_dls(model, x_des, q0,
           lambda_init=1e-2,
           eps_task=1e-4,
           max_iters=100,
           step_max=0.2):
    """
    Damped least-squares IK with simple joint limits and step-size control.
    Returns (q_star, success, info_dict).
    """
    q = model.clip_to_limits(np.array(q0, dtype=float))
    lam = lambda_init
    history = []

    for k in range(max_iters):
        x = model.fk(q)
        e = x_des - x
        err = np.linalg.norm(e)
        history.append({"iter": k, "err": err, "q": q.copy(), "lambda": lam})

        if err <= eps_task:
            return q, True, {"history": history}

        J = model.jacobian(q)
        JTJ = J.T @ J
        H = JTJ + (lam ** 2) * np.eye(J.shape[1])
        g = J.T @ e

        # Solve H delta_q = g
        try:
            delta_q = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            # Increase damping if numerical issues
            lam *= 10.0
            continue

        # Step-size control
        step_norm = np.linalg.norm(delta_q)
        if step_norm > step_max:
            delta_q *= step_max / step_norm

        q_new = model.clip_to_limits(q + delta_q)

        # If clipping was severe, increase damping
        if np.linalg.norm(q_new - (q + delta_q)) > 1e-6:
            lam *= 2.0
        else:
            # If progress is good, we can try reducing damping
            if err > 1e-6 and np.linalg.norm(model.fk(q_new) - x_des) < err:
                lam = max(lam * 0.7, 1e-4)

        q = q_new

    return q, False, {"history": history}


if __name__ == "__main__":
    # Example usage
    model = Planar2RModel(L1=0.5, L2=0.4)
    x_des = np.array([0.6, 0.3])

    # Add synthetic sensor noise to target
    noise = 0.005 * np.random.randn(2)
    x_noisy = x_des + noise

    q0 = np.array([0.0, 0.0])
    q_star, success, info = ik_dls(model, x_noisy, q0)

    print("Success:", success)
    print("q_star:", q_star)
    print("End-effector (noisy target):", x_noisy)
    print("End-effector (achieved):   ", model.fk(q_star))
      
