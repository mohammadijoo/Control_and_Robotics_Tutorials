#include <iostream>
#include <Eigen/Dense>

int main() {
  Eigen::Matrix2d A;
  A << 0.0, 1.0,
       -2.0, -3.0;

  Eigen::EigenSolver<Eigen::Matrix2d> es(A);
  std::cout << "eig(A) =\n" << es.eigenvalues() << std::endl;

  return 0;
}
      
