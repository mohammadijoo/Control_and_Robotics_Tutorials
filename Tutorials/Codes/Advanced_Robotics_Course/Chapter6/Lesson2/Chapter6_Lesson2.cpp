#include <iostream>
#include <cmath>

struct GaussianBelief1D {
    double mu;
    double sigma2;

    GaussianBelief1D(double mu_in = 0.0, double sigma2_in = 1.0)
        : mu(mu_in), sigma2(sigma2_in) {}

    GaussianBelief1D predict(double u, double sigma_w2) const {
        double mu_pred = mu + u;
        double sigma2_pred = sigma2 + sigma_w2;
        return GaussianBelief1D(mu_pred, sigma2_pred);
    }

    GaussianBelief1D update(double z, double sigma_v2) const {
        double K = sigma2 / (sigma2 + sigma_v2);
        double mu_post = mu + K * (z - mu);
        double sigma2_post = (1.0 - K) * sigma2;
        return GaussianBelief1D(mu_post, sigma2_post);
    }
};

double one_step_cost(const GaussianBelief1D& b,
                     double u,
                     double x_goal,
                     double sigma_w2,
                     double lambda_unc) {
    GaussianBelief1D pred = b.predict(u, sigma_w2);
    double mu_err = pred.mu - x_goal;
    return mu_err * mu_err + lambda_unc * pred.sigma2;
}

double choose_control(const GaussianBelief1D& b,
                      const double* actions,
                      std::size_t n_actions,
                      double x_goal,
                      double sigma_w2,
                      double lambda_unc) {
    double best_u = actions[0];
    double best_cost = one_step_cost(b, best_u, x_goal, sigma_w2, lambda_unc);
    for (std::size_t i = 1; i < n_actions; ++i) {
        double u = actions[i];
        double cost = one_step_cost(b, u, x_goal, sigma_w2, lambda_unc);
        if (cost < best_cost) {
            best_cost = cost;
            best_u = u;
        }
    }
    return best_u;
}

int main() {
    GaussianBelief1D b(0.0, 1.0);
    double actions[3] = {-1.0, 0.0, 1.0};
    double x_goal = 5.0;
    double sigma_w2 = 0.1;
    double lambda_unc = 0.5;

    double u_star = choose_control(b, actions, 3, x_goal, sigma_w2, lambda_unc);
    std::cout << "Chosen control: " << u_star << std::endl;
    return 0;
}
      
