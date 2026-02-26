#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <Eigen/Dense>

using Eigen::VectorXd;

double logistic(double u) {
    return 1.0 / (1.0 + std::exp(-u));
}

double measurement_likelihood(int z, double theta, double a, double kappa = 4.0) {
    double p1 = logistic(kappa * std::cos(theta - a));
    return (z == 1) ? p1 : (1.0 - p1);
}

VectorXd normalize(const VectorXd &b) {
    double s = b.sum();
    if (s <= 0.0) {
        return VectorXd::Ones(b.size()) / static_cast<double>(b.size());
    }
    return b / s;
}

double entropy(const VectorXd &b) {
    double H = 0.0;
    for (int i = 0; i < b.size(); ++i) {
        double bi = b(i);
        if (bi > 0.0) {
            H -= bi * std::log(bi);
        }
    }
    return H;
}

double expected_entropy_after_action(const VectorXd &theta_grid,
                                     const VectorXd &b,
                                     double a,
                                     double kappa = 4.0) {
    const int n = theta_grid.size();
    double H_exp = 0.0;
    for (int z = 0; z <= 1; ++z) {
        VectorXd like(n);
        for (int i = 0; i < n; ++i) {
            like(i) = measurement_likelihood(z, theta_grid(i), a, kappa);
        }
        VectorXd b_unnorm = like.cwiseProduct(b);
        double pz = b_unnorm.sum();
        if (pz > 0.0) {
            VectorXd b_post = b_unnorm / pz;
            H_exp += pz * entropy(b_post);
        }
    }
    return H_exp;
}

int main() {
    const int n_theta = 360;
    VectorXd theta_grid(n_theta);
    for (int i = 0; i < n_theta; ++i) {
        theta_grid(i) = -M_PI + 2.0 * M_PI * static_cast<double>(i) / n_theta;
    }

    VectorXd b = VectorXd::Ones(n_theta) / static_cast<double>(n_theta);

    // Candidate views
    const int n_views = 12;
    std::vector<double> views(n_views);
    for (int i = 0; i < n_views; ++i) {
        views[i] = -M_PI + 2.0 * M_PI * static_cast<double>(i) / n_views;
    }

    // Ground truth
    double theta_true = 0.7;

    std::mt19937 gen(0);
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    for (int t = 0; t < 6; ++t) {
        double H_prior = entropy(b);
        double best_a = views[0];
        double best_ig = -std::numeric_limits<double>::infinity();

        for (double a : views) {
            double H_post = expected_entropy_after_action(theta_grid, b, a);
            double ig = H_prior - H_post;
            if (ig > best_ig) {
                best_ig = ig;
                best_a = a;
            }
        }

        // Simulate measurement
        double p1 = measurement_likelihood(1, theta_true, best_a);
        int z = (unif(gen) < p1) ? 1 : 0;

        // Update belief
        VectorXd like(n_theta);
        for (int i = 0; i < n_theta; ++i) {
            like(i) = measurement_likelihood(z, theta_grid(i), best_a);
        }
        b = normalize(like.cwiseProduct(b));

        std::cout << "Step " << t
                  << " a=" << best_a
                  << " z=" << z
                  << " IG=" << best_ig
                  << " H=" << entropy(b)
                  << std::endl;
    }

    return 0;
}
      
