import numpy as np

class ClothModel:
    def __init__(self, vertices, edges, rest_lengths, key_ids, key_targets,
                 k_spring=1.0, alpha_def=1.0, beta_ctrl=1e-2):
        """
        vertices: (N,3) initial positions
        edges: list of (i,j) index pairs
        rest_lengths: array of shape (len(edges),)
        key_ids: indices of key vertices K
        key_targets: (len(K),3) desired folded positions
        """
        self.x0 = vertices.copy()
        self.edges = edges
        self.rest_lengths = rest_lengths
        self.key_ids = np.asarray(key_ids, dtype=int)
        self.key_targets = np.asarray(key_targets, dtype=float)
        self.k = k_spring
        self.alpha = alpha_def
        self.beta = beta_ctrl

    def deformation_energy_and_grad(self, x):
        """Return E_def(x) and grad wrt x (flattened)."""
        N = x.shape[0]
        grad = np.zeros_like(x)
        E = 0.0
        for idx, (i, j) in enumerate(self.edges):
            pi = x[i]
            pj = x[j]
            dij = pi - pj
            rij = np.linalg.norm(dij)
            if rij < 1e-8:
                continue
            lij = self.rest_lengths[idx]
            diff = rij - lij
            E_ij = self.k * diff * diff
            E += E_ij
            # gradient
            scale = 2.0 * self.k * diff / rij
            g = scale * dij
            grad[i] += g
            grad[j] -= g
        return E, grad.reshape(-1)

    def terminal_cost_and_grad(self, x_T):
        """Phi_cloth(x_T) and grad wrt x_T (flattened)."""
        grad = np.zeros_like(x_T)
        diff = x_T[self.key_ids] - self.key_targets
        cost = np.sum(diff * diff)
        # gradient: 2 * (p_k - p_k*)
        for local_idx, vid in enumerate(self.key_ids):
            grad[vid] = 2.0 * diff[local_idx]
        return cost, grad.reshape(-1)

def fold_cloth(model, grasp_ids, T=20, step=1e-2, max_iters=200):
    """
    Optimize grasp trajectories for a single fold.
    grasp_ids: indices of grasped vertices controlled by the robot
    T: horizon length
    step: gradient step size
    """
    N = model.x0.shape[0]
    grasp_ids = np.asarray(grasp_ids, dtype=int)
    n_g = len(grasp_ids)

    # Parameterization: delta positions for each grasp over horizon (T, n_g, 3)
    U = np.zeros((T, n_g, 3), dtype=float)

    def rollout(U):
        # Simple quasi-static model: at each step, move grasp vertices, then relax cloth
        x = model.x0.copy()
        xs = [x.copy()]
        for t in range(T):
            for g_idx, vid in enumerate(grasp_ids):
                x[vid] += U[t, g_idx]
            # one gradient step of energy relaxation
            _, gradE = model.deformation_energy_and_grad(x)
            gradE = gradE.reshape(N, 3)
            x -= 0.1 * gradE  # projected gradient step
            xs.append(x.copy())
        return xs

    def objective_and_grad(U):
        xs = rollout(U)
        total_cost = 0.0
        gradU = np.zeros_like(U)

        # terminal cost
        Phi, gradT = model.terminal_cost_and_grad(xs[-1])
        total_cost += Phi

        # backpropagate terminal gradient through last relaxation step (very rough)
        # Here we ignore the dependence of relaxation on U for simplicity.

        # stage costs
        for t in range(len(xs) - 1):
            x_t = xs[t]
            E_def, _ = model.deformation_energy_and_grad(x_t)
            u_t = U[t].reshape(-1)
            total_cost += model.alpha * E_def + model.beta * np.dot(u_t, u_t)
            # d/dU of beta * ||u_t||^2 = 2 * beta * u_t
            gradU[t] += 2.0 * model.beta * U[t]

        return total_cost, gradU

    for it in range(max_iters):
        cost, gU = objective_and_grad(U)
        U -= step * gU
        if it % 20 == 0:
            print(f"iter {it}, cost {cost:.4f}")
    return U
      
