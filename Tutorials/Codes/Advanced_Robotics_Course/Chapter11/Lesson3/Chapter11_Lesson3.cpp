#include <vector>
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

struct Transition {
    int s;
    int a;
};

struct Demonstration {
    std::vector<Transition> traj;
};

VectorXd phi(int s, int a, int k);

// Empirical discounted feature expectation from demonstrations
VectorXd empiricalFeatureExpectation(const std::vector<Demonstration>& demos,
                                     double gamma,
                                     int k) {
    VectorXd mu_E = VectorXd::Zero(k);
    for (const auto& demo : demos) {
        double discount = 1.0;
        for (std::size_t t = 0; t < demo.traj.size(); ++t) {
            int s = demo.traj[t].s;
            int a = demo.traj[t].a;
            mu_E += discount * phi(s, a, k);
            discount *= gamma;
        }
    }
    mu_E /= static_cast<double>(demos.size());
    return mu_E;
}

// Simple gradient update for theta in linear reward R_theta(s,a) = theta^T phi(s,a)
void gradientUpdate(VectorXd& theta,
                    const VectorXd& mu_E,
                    const VectorXd& mu_theta,
                    double alpha) {
    VectorXd grad = mu_E - mu_theta;
    theta += alpha * grad;
}
      
