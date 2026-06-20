import numpy as np

class GaussianBelief1D:
    def __init__(self, mu, sigma2):
        self.mu = float(mu)
        self.sigma2 = float(sigma2)

    def predict(self, u, sigma_w2):
        """
        Linear 1D dynamics: x_{t+1} = x_t + u + w_t
        """
        mu_pred = self.mu + float(u)
        sigma2_pred = self.sigma2 + float(sigma_w2)
        return GaussianBelief1D(mu_pred, sigma2_pred)

    def update(self, z, sigma_v2):
        """
        1D Kalman update using measurement z = x + v.
        """
        sigma2_pred = self.sigma2
        K = sigma2_pred / (sigma2_pred + float(sigma_v2))
        mu_post = self.mu + K * (float(z) - self.mu)
        sigma2_post = (1.0 - K) * sigma2_pred
        return GaussianBelief1D(mu_post, sigma2_post)

def one_step_cost(belief, u, x_goal, sigma_w2, lam):
    """
    Approximate one-step belief-space cost using predicted belief,
    ignoring the update step (open-loop approximation).
    """
    pred = belief.predict(u, sigma_w2)
    mu_err = pred.mu - float(x_goal)
    return mu_err * mu_err + lam * pred.sigma2

def choose_control(belief, actions, x_goal, sigma_w2, lam):
    best_u = None
    best_cost = float("inf")
    for u in actions:
        cost = one_step_cost(belief, u, x_goal, sigma_w2, lam)
        if cost < best_cost:
            best_cost = cost
            best_u = u
    return best_u, best_cost

if __name__ == "__main__":
    # Current belief: x ~ N(mu, sigma2)
    belief = GaussianBelief1D(mu=0.0, sigma2=1.0)
    x_goal = 5.0
    sigma_w2 = 0.1
    lam = 0.5
    actions = [-1.0, 0.0, 1.0]

    u_star, J_star = choose_control(belief, actions, x_goal, sigma_w2, lam)
    print("Chosen control:", u_star)
    print("Approximate one-step cost:", J_star)
      
