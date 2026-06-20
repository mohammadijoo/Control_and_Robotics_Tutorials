#include <iostream>
#include <complex>
#include <vector>
#include <Eigen/Dense>

using cd = std::complex<double>;
using Eigen::MatrixXd;
using Eigen::VectorXd;

cd freqresp(const MatrixXd& A, const VectorXd& B, const VectorXd& C, double D, double w) {
  // Computes scalar SISO G(jw) = C*(jwI - A)^(-1)*B + D
  const int n = A.rows();
  Eigen::Matrix<cd, Eigen::Dynamic, Eigen::Dynamic> M(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      M(i,j) = cd(0.0, 0.0) - cd(A(i,j), 0.0);
    }
    M(i,i) += cd(0.0, w); // add jw on diagonal: jwI - A
  }

  Eigen::Matrix<cd, Eigen::Dynamic, 1> Bb(n);
  Eigen::Matrix<cd, 1, Eigen::Dynamic> Cc(n);
  for (int i = 0; i < n; ++i) {
    Bb(i) = cd(B(i), 0.0);
    Cc(i) = cd(C(i), 0.0);
  }

  Eigen::Matrix<cd, Eigen::Dynamic, 1> x = M.fullPivLu().solve(Bb);
  cd y = Cc * x;
  return y + cd(D, 0.0);
}

int main() {
  // Minimal system for G(s)=1/(s+2):
  // xdot = -2 x + 1 u, y = 1 x
  MatrixXd A1(1,1); A1 << -2.0;
  VectorXd B1(1);   B1 <<  1.0;
  VectorXd C1(1);   C1 <<  1.0;
  double D1 = 0.0;

  // Nonminimal augmented system: add hidden state xh with xhdot=-1*xh, no coupling
  MatrixXd A2 = MatrixXd::Zero(2,2);
  A2(0,0) = -2.0; A2(1,1) = -1.0;
  VectorXd B2(2); B2 << 1.0, 0.0;
  VectorXd C2(2); C2 << 1.0, 0.0;
  double D2 = 0.0;

  std::vector<double> ws = {0.01, 0.1, 1.0, 10.0, 100.0};
  for (double w : ws) {
    cd G1 = freqresp(A1, B1, C1, D1, w);
    cd G2 = freqresp(A2, B2, C2, D2, w);
    std::cout << "w=" << w
              << "  G_min=" << G1
              << "  G_nonmin=" << G2
              << "  diff=" << (G1 - G2)
              << std::endl;
  }
  return 0;
}
      
