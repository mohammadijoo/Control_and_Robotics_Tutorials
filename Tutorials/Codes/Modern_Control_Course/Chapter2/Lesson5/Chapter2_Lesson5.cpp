#include <iostream>
#include <Eigen/Dense>

int main() {
  Eigen::Matrix3d A;
  A << 2, 1, 0,
       0, 2, 0,
       0, 0, 3;

  const double lambda = 2.0;
  Eigen::Matrix3d M = A - lambda * Eigen::Matrix3d::Identity();

  // Step 1: find an eigenvector v1 in ker(M).
  // For this simple triangular example, we can pick v1 = [1,0,0]^T (check M*v1 = 0).
  Eigen::Vector3d v1(1, 0, 0);
  std::cout << "M*v1 = " << (M*v1).transpose() << std::endl;

  // Step 2: find a generalized eigenvector v2 solving M*v2 = v1.
  // Solve linear system in least squares / exact solve when consistent.
  Eigen::Vector3d v2 = M.fullPivLu().solve(v1);
  std::cout << "v2 = " << v2.transpose() << std::endl;
  std::cout << "M*v2 = " << (M*v2).transpose() << " (should equal v1)" << std::endl;

  // For the eigenvalue 3, an eigenvector is e3 = [0,0,1]^T.
  Eigen::Vector3d w(0, 0, 1);

  // Assemble P = [v1 v2 w] and compute J = P^{-1} A P
  Eigen::Matrix3d P;
  P.col(0) = v1;
  P.col(1) = v2;
  P.col(2) = w;

  Eigen::Matrix3d J = P.inverse() * A * P;
  std::cout << "\nJ = \n" << J << std::endl;

  return 0;
}
