
#include <iostream>
#include <vector>
#include <Eigen/Dense>

int main() {
    double b = 0.2;
    double k_p = 25.0;
    double k_d = 10.0;

    auto A_matrix = [&] (double J) {
        Eigen::Matrix2d A;
        A << 0.0,        1.0,
              -k_p / J,  -(b + k_d) / J;
        return A;
    };

    std::vector<double> J_values;
    for (int i = 0; i <= 20; ++i) {
        double J = 0.5 + (1.0 / 20.0) * i; // from 0.5 to 1.5
        J_values.push_back(J);
    }

    bool robust_stable = true;
    for (double J : J_values) {
        Eigen::Matrix2d A = A_matrix(J);
        Eigen::EigenSolver<Eigen::Matrix2d> es(A);
        Eigen::VectorXcd lam = es.eigenvalues();
        double max_real = lam.real().maxCoeff();
        if (max_real >= 0.0) {
            std::cout << "Unstable or marginally stable at J = "
                      << J << std::endl;
            std::cout << "Eigenvalues: " << lam.transpose() << std::endl;
            robust_stable = false;
            break;
        }
    }

    std::cout << "Robust stability on grid: "
              << (robust_stable ? "true" : "false") << std::endl;
    return 0;
}
