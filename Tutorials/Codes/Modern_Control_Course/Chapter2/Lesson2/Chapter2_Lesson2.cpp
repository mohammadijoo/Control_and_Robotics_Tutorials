#include <iostream>
#include <Eigen/Dense>

int main() {
  // T: R^3 -> R^2 with matrix A
  Eigen::Matrix<double, 2, 3> A;
  A << 1, 2, 0,
       0,-1, 3;

  Eigen::Vector3d x;
  x << 1.0, -2.0, 0.5;

  Eigen::Vector2d Tx = A * x;

  std::cout << "A=\n" << A << "\n";
  std::cout << "x=\n" << x << "\n";
  std::cout << "T(x)=A x=\n" << Tx << "\n";

  // Composition with S: R^2 -> R^2
  Eigen::Matrix2d B;
  B << 0, 1,
      -1, 0;

  Eigen::Matrix<double, 2, 3> BA = B * A;
  std::cout << "Composite BA=\n" << BA << "\n";
  std::cout << "(S o T)(x)=BA x=\n" << (BA * x) << "\n";

  return 0;
}
      
