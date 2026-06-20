#include <iostream>
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

VectorXd regressor(double q, double qd, double qdd) {
    VectorXd Y(4);
    Y(0) = qdd;
    Y(1) = qd;
    Y(2) = (qd > 0.0) ? 1.0 : ((qd < 0.0) ? -1.0 : 0.0);
    Y(3) = std::sin(q);
    return Y;
}

int main() {
    VectorXd theta_true(4), theta_nominal(4);
    theta_true << 0.8, 0.05, 0.2, 3.0;
    theta_nominal << 1.0, 0.02, 0.1, 2.5;

    double q = 0.5;      // example state
    double qd = 0.1;
    double qdd = -0.3;

    VectorXd Y = regressor(q, qd, qdd);
    double tau_true = Y.dot(theta_true);
    double tau_nominal = Y.dot(theta_nominal);

    std::cout << "tau_true = " << tau_true << std::endl;
    std::cout << "tau_nominal = " << tau_nominal << std::endl;
    std::cout << "difference = " << (tau_nominal - tau_true) << std::endl;
    return 0;
}
      
