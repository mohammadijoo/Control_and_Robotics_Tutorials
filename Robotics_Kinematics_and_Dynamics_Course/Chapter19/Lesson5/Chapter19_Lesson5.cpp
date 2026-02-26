#include <iostream>
#include <vector>
#include <Eigen/Dense>

// For full robots, you could also include RBDL:
// #include <rbdl/rbdl.h>

int main() {
    using Eigen::VectorXd;
    using Eigen::MatrixXd;

    const double Ts = 0.002;
    const double Tend = 8.0;
    const int N = static_cast<int>(Tend / Ts);

    std::vector<double> t(N);
    for (int k = 0; k < N; ++k) {
        t[k] = k * Ts;
    }

    // Generate exciting trajectory
    VectorXd q(N), dq(N), ddq(N);
    const double pi = 3.141592653589793;
    for (int k = 0; k < N; ++k) {
        double tk = t[k];
        q(k) = 0.6 * std::sin(2.0 * pi * 0.4 * tk)
             + 0.4 * std::sin(2.0 * pi * 0.9 * tk);
        dq(k) = 0.6 * 2.0 * pi * 0.4 * std::cos(2.0 * pi * 0.4 * tk)
              + 0.4 * 2.0 * pi * 0.9 * std::cos(2.0 * pi * 0.9 * tk);
        ddq(k) = -0.6 * std::pow(2.0 * pi * 0.4, 2) * std::sin(2.0 * pi * 0.4 * tk)
               - 0.4 * std::pow(2.0 * pi * 0.9, 2) * std::sin(2.0 * pi * 0.9 * tk);
    }

    // True parameters
    VectorXd pi_true(3);
    pi_true << 0.35, 2.1, 0.08;

    // Build regressor and torque
    MatrixXd Y(N, 3);
    VectorXd tau(N);
    for (int k = 0; k < N; ++k) {
        Y(k, 0) = ddq(k);
        Y(k, 1) = std::cos(q(k));
        Y(k, 2) = dq(k);
    }
    tau = Y * pi_true;

    // Least-squares estimate: pi_hat = (Y^T Y)^(-1) Y^T tau
    MatrixXd YTY = Y.transpose() * Y;
    VectorXd YTtau = Y.transpose() * tau;
    VectorXd pi_hat = YTY.ldlt().solve(YTtau);

    std::cout << "pi_hat = " << pi_hat.transpose() << std::endl;
    return 0;
}
      
