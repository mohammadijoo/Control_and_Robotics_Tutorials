#include <iostream>
#include <Eigen/Dense>

int main() {
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  MatrixXd A(3,3), B(3,1), C(1,3), D(1,1);
  A << 0.0, 1.0, 0.0,
       0.0, 0.0, 1.0,
      -1.0,-10.0,-1000.0;

  B << 0.0, 0.0, 1.0;
  C << 1.0, 0.0, 0.0;
  D << 0.0;

  double omega = 10.0;
  MatrixXd S = MatrixXd::Zero(3,3);
  S(0,0) = 1.0;
  S(1,1) = omega;
  S(2,2) = omega*omega;

  MatrixXd Sinv = S.inverse();

  MatrixXd As = Sinv * A * S;
  MatrixXd Bs = Sinv * B;
  MatrixXd Cs = C * S;
  MatrixXd Ds = D;

  std::cout << "As =\n" << As << "\n\n";
  std::cout << "Bs =\n" << Bs << "\n\n";
  std::cout << "Cs =\n" << Cs << "\n\n";

  // Example: map a scaled state z back to physical x = S z
  VectorXd z(3);
  z << 0.1, -0.2, 0.05;
  VectorXd x = S * z;
  std::cout << "x = S z =\n" << x << "\n";

  return 0;
}
