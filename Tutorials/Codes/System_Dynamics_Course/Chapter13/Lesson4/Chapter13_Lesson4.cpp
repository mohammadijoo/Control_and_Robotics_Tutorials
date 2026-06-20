/*
Chapter13_Lesson4.cpp
Damping in MDOF Systems and Mode Shapes with Damping

Demonstrates:
1) Undamped generalized eigenproblem: K phi = (w^2) M phi
2) Rayleigh damping: C = alpha*M + beta*K -> diagonal modal damping matrix
3) Non-proportional damping: state-space eigenanalysis -> complex poles

Dependencies:
  - Eigen (https://eigen.tuxfamily.org/)
Build (example):
  g++ -O2 -std=c++17 Chapter13_Lesson4.cpp -I /path/to/eigen -o Chapter13_Lesson4
*/

#include <iostream>
#include <vector>
#include <complex>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static void chain_matrices_3dof(MatrixXd& M, MatrixXd& K) {
    const double m1 = 1.2, m2 = 1.0, m3 = 0.8;
    const double k1 = 2500.0, k2 = 1800.0, k3 = 1200.0;

    M = MatrixXd::Zero(3,3);
    M(0,0) = m1; M(1,1) = m2; M(2,2) = m3;

    K = MatrixXd::Zero(3,3);
    K(0,0) = k1 + k2; K(0,1) = -k2;
    K(1,0) = -k2;     K(1,1) = k2 + k3; K(1,2) = -k3;
    K(2,1) = -k3;     K(2,2) = k3;
}

static MatrixXd mass_normalize_modes(const MatrixXd& M, const MatrixXd& Phi) {
    MatrixXd PhiN = Phi;
    for (int i=0; i<Phi.cols(); ++i) {
        double mi = Phi.col(i).transpose() * M * Phi.col(i);
        PhiN.col(i) /= std::sqrt(mi);
    }
    return PhiN;
}

static void rayleigh_from_two_targets(double omega_i, double zeta_i,
                                      double omega_j, double zeta_j,
                                      double& alpha, double& beta) {
    // alpha + beta*omega^2 = 2*zeta*omega at two omegas
    Eigen::Matrix2d A;
    A << 1.0, omega_i*omega_i,
         1.0, omega_j*omega_j;
    Eigen::Vector2d b;
    b << 2.0*zeta_i*omega_i,
         2.0*zeta_j*omega_j;
    Eigen::Vector2d x = A.fullPivLu().solve(b);
    alpha = x(0);
    beta  = x(1);
}

static MatrixXd build_state_matrix(const MatrixXd& M, const MatrixXd& C, const MatrixXd& K) {
    const int n = static_cast<int>(M.rows());
    MatrixXd Minv = M.inverse();

    MatrixXd A = MatrixXd::Zero(2*n, 2*n);
    A.block(0, n, n, n) = MatrixXd::Identity(n, n);
    A.block(n, 0, n, n) = -Minv * K;
    A.block(n, n, n, n) = -Minv * C;
    return A;
}

int main() {
    MatrixXd M, K;
    chain_matrices_3dof(M, K);

    // Undamped generalized eigenproblem (symmetric):
    // Use K' = L^{-1} K L^{-T} with M = L L^T (Cholesky), then eig(K')
    Eigen::LLT<MatrixXd> llt(M);
    MatrixXd L = llt.matrixL();
    MatrixXd Linv = L.inverse();
    MatrixXd Kt = Linv * K * Linv.transpose(); // symmetric

    Eigen::SelfAdjointEigenSolver<MatrixXd> es(Kt);
    VectorXd lam = es.eigenvalues();           // omega^2
    MatrixXd Y = es.eigenvectors();            // in transformed coordinates
    VectorXd omega = lam.array().sqrt();

    // Back-transform mode shapes: phi = L^{-T} y
    MatrixXd Phi = L.transpose().inverse() * Y;
    Phi = mass_normalize_modes(M, Phi);

    std::cout << "Undamped natural frequencies (rad/s):\n" << omega.transpose() << "\n\n";
    std::cout << "Phi^T M Phi (should be I):\n" << (Phi.transpose()*M*Phi) << "\n\n";
    std::cout << "Phi^T K Phi (should be diag(omega^2)):\n" << (Phi.transpose()*K*Phi) << "\n\n";

    // Rayleigh damping parameters from two target zetas (mode 1 and 3)
    double alpha=0.0, beta=0.0;
    rayleigh_from_two_targets(omega(0), 0.02, omega(2), 0.05, alpha, beta);
    MatrixXd C_ray = alpha*M + beta*K;
    MatrixXd Cm_ray = Phi.transpose() * C_ray * Phi;

    std::cout << "Rayleigh coefficients: alpha=" << alpha << ", beta=" << beta << "\n";
    std::cout << "Modal damping matrix Phi^T C Phi (Rayleigh) ~ diagonal:\n" << Cm_ray << "\n\n";

    // Modal damping ratios: zeta_r = 0.5*(alpha/omega_r + beta*omega_r)
    VectorXd zeta = 0.5*(alpha*omega.cwiseInverse() + beta*omega);
    std::cout << "Modal damping ratios (Rayleigh):\n" << zeta.transpose() << "\n\n";

    // Non-proportional damping example: damper between DOF1 and DOF3, plus damper to ground at DOF2
    MatrixXd C_np = MatrixXd::Zero(3,3);
    double c13 = 45.0;
    double c2g = 35.0;
    C_np(0,0) += c13; C_np(2,2) += c13;
    C_np(0,2) -= c13; C_np(2,0) -= c13;
    C_np(1,1) += c2g;

    MatrixXd Cm_np = Phi.transpose() * C_np * Phi;
    std::cout << "Non-proportional C:\n" << C_np << "\n\n";
    std::cout << "Phi^T C Phi (non-proportional) has off-diagonals:\n" << Cm_np << "\n\n";

    // State-space eigenanalysis for complex poles
    MatrixXd A = build_state_matrix(M, C_np, K);
    Eigen::EigenSolver<MatrixXd> ces(A);
    Eigen::VectorXcd evals = ces.eigenvalues();

    // Print a few poles with positive imaginary part (one per conjugate pair)
    std::vector<std::complex<double>> poles;
    poles.reserve(evals.size());
    for (int i=0; i<evals.size(); ++i) {
        if (std::imag(evals(i)) > 1e-8) poles.push_back(evals(i));
    }
    std::sort(poles.begin(), poles.end(),
              [](auto a, auto b){ return std::imag(a) < std::imag(b); });

    std::cout << "Complex poles (sigma + j*wd), first three modes:\n";
    for (int i=0; i<std::min<int>(3, (int)poles.size()); ++i) {
        double sigma = std::real(poles[i]);
        double wd = std::imag(poles[i]);
        double wn = std::sqrt(sigma*sigma + wd*wd);
        double zeta_np = (wn > 0.0) ? (-sigma/wn) : 0.0;
        std::cout << "Mode " << (i+1) << ": sigma=" << sigma << ", wd=" << wd
                  << ", wn=" << wn << ", zeta=" << zeta_np << "\n";
    }

    return 0;
}
