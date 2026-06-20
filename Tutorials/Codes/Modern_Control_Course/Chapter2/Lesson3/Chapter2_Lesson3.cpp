#include <iostream>
#include <Eigen/Dense>

int main() {
  Eigen::MatrixXd A(3,3);
  A << 4.0, 1.0, 0.0,
       1.0, 3.0, 1.0,
       0.0, 1.0, 2.0;

  // 1) Symmetric eigendecomposition (spectral theorem scenario)
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
  if (es.info() != Eigen::Success) {
    std::cerr << "Eigen decomposition failed." << std::endl;
    return 1;
  }

  std::cout << "Eigenvalues:\n" << es.eigenvalues() << "\n";
  std::cout << "Eigenvectors (columns):\n" << es.eigenvectors() << "\n";

  // 2) Reconstruct A = Q Λ Q^T
  Eigen::MatrixXd Q = es.eigenvectors();
  Eigen::MatrixXd Lambda = es.eigenvalues().asDiagonal();
  Eigen::MatrixXd Arec = Q * Lambda * Q.transpose();
  std::cout << "Reconstruction error (Frobenius): "
            << (A - Arec).norm() << "\n";

  // 3) Power iteration from scratch
  Eigen::VectorXd x = Eigen::VectorXd::Random(3);
  x.normalize();

  double lam_old = 0.0;
  const int max_iter = 2000;
  const double tol = 1e-10;

  for (int k = 0; k < max_iter; ++k) {
    Eigen::VectorXd y = A * x;
    x = y.normalized();

    double lam = x.transpose() * A * x;  // Rayleigh estimate (best for symmetric A)
    if (std::abs(lam - lam_old) < tol) {
      std::cout << "Power iteration converged in " << (k+1) << " iterations.\n";
      std::cout << "lambda_hat = " << lam << "\n";
      std::cout << "v_hat^T = " << x.transpose() << "\n";
      break;
    }
    lam_old = lam;
  }

  return 0;
}
