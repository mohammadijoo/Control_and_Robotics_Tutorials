#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

double normalQuantile(double p) {
    // In practice use boost::math or another reliable implementation.
    // Here we assume access to std::erfcinv (C++17 extension or vendor-specific).
    // Phi^{-1}(p) = -sqrt(2) * erfcinv(2p)
    return -std::sqrt(2.0) * std::erfcinv(2.0 * p);
}

bool chanceConstraintSatisfied(
    const Eigen::VectorXd& mu,
    const Eigen::MatrixXd& Sigma,
    const Eigen::VectorXd& a,
    double b,
    double epsilon
) {
    double m = a.transpose() * mu;
    double s2 = (a.transpose() * Sigma * a)(0, 0);
    if (s2 < 0.0) {
        std::cerr << "Warning: negative variance, clipping to zero\n";
        s2 = 0.0;
    }
    double s = std::sqrt(s2);
    double beta = normalQuantile(1.0 - epsilon);
    // Deterministic equivalent: m + beta * s <= b
    return (m + beta * s) <= b + 1e-9;
}

int main() {
    // Example: 2D state [p; v], double integrator covariance at one time step
    Eigen::Vector2d mu;
    mu << 1.0, 0.2;
    Eigen::Matrix2d Sigma;
    Sigma << 0.05 * 0.05, 0.0,
             0.0,         0.02 * 0.02;

    Eigen::Vector2d a;
    a << 1.0, 0.0; // constrain position only
    double p_max = 2.0;
    double epsilon = 0.01;

    bool ok = chanceConstraintSatisfied(mu, Sigma, a, p_max, epsilon);
    std::cout << "Chance constraint satisfied? " << (ok ? "yes" : "no") << std::endl;

    return 0;
}
      
