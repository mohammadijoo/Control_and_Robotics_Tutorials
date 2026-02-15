#include <iostream>
#include <Eigen/Dense>

int main() {
    // Symmetric "inertia" matrix (for a link in its body frame)
    Eigen::Matrix3d I;
    I << 0.20, 0.01, 0.00,
          0.01, 0.15, 0.00,
          0.00, 0.00, 0.10;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(I);
    if (solver.info() != Eigen::Success) {
        std::cerr << "Eigen decomposition failed\n";
        return 1;
    }

    std::cout << "Eigenvalues (principal moments):\n"
              << solver.eigenvalues() << "\n\n";
    std::cout << "Eigenvectors (principal axes as columns):\n"
              << solver.eigenvectors() << "\n";

    // Check A v = lambda v for the first eigenpair
    Eigen::Vector3d v0 = solver.eigenvectors().col(0);
    double lambda0 = solver.eigenvalues()(0);
    Eigen::Vector3d lhs = I * v0;
    Eigen::Vector3d rhs = lambda0 * v0;
    std::cout << "Residual norm for first eigenpair: "
              << (lhs - rhs).norm() << "\n";
    return 0;
}
      
