#include <iostream>
#include <Eigen/Dense>

int main() {
  Eigen::Matrix2d A;
  A << 1.0, 0.01,
       0.0, 1.0;
  Eigen::Vector2d B;
  B << 0.0, 0.01;
  Eigen::RowVector2d K;
  K << 2.0, 0.5;

  Eigen::Vector2d x(0.1, 0.0);

  for (int k = 0; k < 1000; ++k) {
    double u = -(K * x)(0);
    x = A * x + B * u;
  }
  std::cout << x.transpose() << std::endl;
  return 0;
}
