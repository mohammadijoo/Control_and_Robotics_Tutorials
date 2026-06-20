#include <iostream>
#include <vector>
#include <complex>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

int main() {
  using Eigen::MatrixXd;
  using Eigen::VectorXcd;

  MatrixXd A(2,2); A << 0, 1,
                         -2, -3;
  MatrixXd B(2,1); B << 0,
                         1;
  MatrixXd C(1,2); C << 1, 1;
  MatrixXd D(1,1); D << 0;

  // Poles: eigenvalues of A
  Eigen::EigenSolver<MatrixXd> esA(A);
  VectorXcd poles = esA.eigenvalues();
  std::cout << "eig(A) poles:\n" << poles << "\n\n";

  // Build Rosenbrock pencil for SISO:
  // P(s) = [[sI-A, -B],[C, D]] = s*E - F
  // E = [[I, 0],[0, 0]], F = [[A, B],[-C, -D]]
  int n = 2;
  MatrixXd E = MatrixXd::Zero(n+1, n+1);
  E.block(0,0,n,n) = MatrixXd::Identity(n,n);

  MatrixXd F = MatrixXd::Zero(n+1, n+1);
  F.block(0,0,n,n) = A;
  F.block(0,n,n,1) = B;
  F.block(n,0,1,n) = -C;
  F(n,n) = -D(0,0);

  // Generalized eigenvalues: F v = lambda E v
  Eigen::GeneralizedEigenSolver<MatrixXd> ges(F, E);
  VectorXcd lambdas = ges.eigenvalues();

  std::cout << "Generalized eigenvalues (finite zeros appear among these):\n";
  for (int i=0; i<lambdas.size(); ++i) {
    std::complex<double> lam = lambdas(i);
    // Filter huge magnitudes as "infinite" placeholders (heuristic)
    if (std::abs(lam) < 1e8) {
      std::cout << "  " << lam << "\n";
    }
  }
  return 0;
}
