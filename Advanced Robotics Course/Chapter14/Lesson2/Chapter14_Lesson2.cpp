#include <iostream>
#include <Eigen/Dense>

int main() {
    const int N = 4;
    Eigen::MatrixXd L = N * Eigen::MatrixXd::Identity(N, N)
                      - Eigen::MatrixXd::Ones(N, N);

    double alpha = 0.3; // 0 < alpha < 0.5 for N = 4
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(N, N) - alpha * L;

    Eigen::VectorXd x(N);
    x << 0.0, 2.0, -1.0, 4.0;

    int T = 30;
    for (int k = 0; k < T; ++k) {
        x = P * x;
    }

    std::cout << "Final state: " << x.transpose() << std::endl;
    std::cout << "Average of initial states: "
              << (0.0 + 2.0 - 1.0 + 4.0) / 4.0
              << std::endl;

    return 0;
}
      
