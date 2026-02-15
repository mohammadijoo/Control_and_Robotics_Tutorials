#include <iostream>
#include <vector>
#include <Eigen/Dense>

// Build regressor for 1-DOF pendulum
Eigen::MatrixXd buildRegressorPendulum(const std::vector<double> &q,
                                       const std::vector<double> &qd,
                                       const std::vector<double> &qdd)
{
    const std::size_t N = q.size();
    Eigen::MatrixXd Y(N, 3);
    for (std::size_t k = 0; k < N; ++k) {
        Y(static_cast<int>(k), 0) = qdd[k];
        Y(static_cast<int>(k), 1) = std::sin(q[k]);
        Y(static_cast<int>(k), 2) = qd[k];
    }
    return Y;
}

int main()
{
    // Example: assume q, qd, qdd, tau_meas are filled from logs
    std::vector<double> q, qd, qdd, tau_meas;
    // ... fill data ...

    const std::size_t N = q.size();
    Eigen::MatrixXd Y = buildRegressorPendulum(q, qd, qdd);

    Eigen::VectorXd T(N);
    for (std::size_t k = 0; k < N; ++k) {
        T(static_cast<int>(k)) = tau_meas[k];
    }

    // Solve normal equations using a robust decomposition
    Eigen::MatrixXd YtY = Y.transpose() * Y;
    Eigen::VectorXd YtT = Y.transpose() * T;

    Eigen::Vector3d phi_hat =
        YtY.ldlt().solve(YtT);  // 3 parameters for the pendulum

    std::cout << "Estimated parameters (a1, a2, a3): "
              << phi_hat.transpose() << std::endl;

    return 0;
}
      
