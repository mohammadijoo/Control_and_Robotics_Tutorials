// Chapter 13 - Vibrations and Multi-Degree-of-Freedom (MDOF) Systems
// Lesson 1: Natural Frequencies and Normal Modes (Eigenvalue Problems)
//
// Build:
//   - Requires Eigen (header-only): https://eigen.tuxfamily.org/
//   - Example (g++):
//       g++ -O2 -std=c++17 Chapter13_Lesson1.cpp -I path/to/eigen -o mdof_modes
//
// This program:
//   1) Builds (M, K) for a 3-DOF undamped mass-spring chain.
//   2) Solves K phi = (w^2) M phi using Eigen's GeneralizedSelfAdjointEigenSolver.
//   3) Mass-normalizes modes and verifies Phi^T M Phi = I and Phi^T K Phi = diag(w^2).

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

static void mdofChainMK(const std::vector<double>& masses,
                        const std::vector<double>& springs,
                        Eigen::MatrixXd& M,
                        Eigen::MatrixXd& K)
{
    const int n = static_cast<int>(masses.size());
    if (static_cast<int>(springs.size()) != n) {
        throw std::runtime_error("springs must have length n (k1..kn).");
    }

    M = Eigen::MatrixXd::Zero(n, n);
    K = Eigen::MatrixXd::Zero(n, n);

    for (int i = 0; i < n; ++i) M(i, i) = masses[i];

    for (int i = 0; i < n; ++i) {
        K(i, i) += springs[i];
        if (i > 0) {
            K(i, i)     += springs[i-1];
            K(i, i-1)   -= springs[i-1];
            K(i-1, i)   -= springs[i-1];
        }
    }
}

int main()
{
    try {
        std::vector<double> masses  = {2.0, 1.5, 1.0};
        std::vector<double> springs = {200.0, 300.0, 250.0};

        Eigen::MatrixXd M, K;
        mdofChainMK(masses, springs, M, K);

        Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver(K, M);
        if (solver.info() != Eigen::Success) {
            std::cerr << "Eigen solve failed.\n";
            return 1;
        }

        Eigen::VectorXd w2 = solver.eigenvalues();    // ascending (w^2)
        Eigen::MatrixXd Phi = solver.eigenvectors();  // columns are modes

        // Clean small negative numerical noise
        for (int i = 0; i < w2.size(); ++i) if (w2(i) < 0.0 && std::abs(w2(i)) < 1e-12) w2(i) = 0.0;

        Eigen::VectorXd w(w2.size());
        for (int i = 0; i < w2.size(); ++i) w(i) = std::sqrt(std::max(0.0, w2(i)));

        // Mass-normalize columns: phi_i^T M phi_i = 1
        for (int i = 0; i < Phi.cols(); ++i) {
            double mi = Phi.col(i).transpose() * M * Phi.col(i);
            Phi.col(i) /= std::sqrt(mi);
        }

        std::cout << "Natural frequencies (rad/s):\n";
        for (int i = 0; i < w.size(); ++i) {
            std::cout << "  w" << (i+1) << " = " << w(i) << "\n";
        }

        Eigen::MatrixXd Mt = Phi.transpose() * M * Phi;
        Eigen::MatrixXd Kt = Phi.transpose() * K * Phi;

        std::cout << "\nPhi^T M Phi (should be I):\n" << Mt << "\n";
        std::cout << "\nPhi^T K Phi (should be diag(w^2)):\n" << Kt << "\n";

        // Infinity norm errors
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Mt.rows(), Mt.cols());
        Eigen::MatrixXd D = w2.asDiagonal();
        double errM = (Mt - I).cwiseAbs().maxCoeff();
        double errK = (Kt - D).cwiseAbs().maxCoeff();

        std::cout << "\nErrors (max abs entry): err_M=" << errM << ", err_K=" << errK << "\n";
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
