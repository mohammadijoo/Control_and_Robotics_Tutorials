#include <iostream>
#include <vector>
#include <Eigen/Dense>

int main() {
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  // Original realization (2-state SISO)
  MatrixXd A(2,2);
  A << 0.0, 1.0,
      -2.0, -3.0;

  VectorXd B(2);
  B << 0.0, 1.0;

  Eigen::RowVectorXd C(2);
  C << 2.0, 1.0;

  double D = 0.0;

  // Similarity transform z = T x
  MatrixXd T(2,2);
  T << 1.0, 1.0,
       0.0, 1.0;
  MatrixXd Ti = T.inverse();

  MatrixXd A2 = T * A * Ti;
  VectorXd B2 = T * B;
  Eigen::RowVectorXd C2 = C * Ti;
  double D2 = D;

  auto evalG = [](double s,
                  const MatrixXd& A,
                  const VectorXd& B,
                  const Eigen::RowVectorXd& C,
                  double D) {
    MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
    MatrixXd M = (s * I - A);
    VectorXd x = M.fullPivLu().solve(B);
    double val = (C * x)(0) + D;
    return val;
  };

  std::vector<double> tests = {1.0, 2.0, 3.0};
  for (double s : tests) {
    double g1 = evalG(s, A, B, C, D);
    double g2 = evalG(s, A2, B2, C2, D2);
    std::cout << "s=" << s << "  G1(s)=" << g1 << "  G2(s)=" << g2 << std::endl;
  }

  return 0;
}
      
