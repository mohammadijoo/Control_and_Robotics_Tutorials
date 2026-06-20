#include <iostream>
#include <Eigen/Dense>

int main() {
  Eigen::MatrixXd A(3,4);
  A << 1,2,3,1,
       2,4,6,2,
       1,1,1,0;

  // Rank via SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  Eigen::VectorXd s = svd.singularValues();

  double tol = 1e-10;
  int rank = 0;
  for (int i = 0; i < s.size(); ++i) {
    if (s(i) > tol) rank++;
  }

  int n = A.cols();
  int nullity = n - rank;

  std::cout << "rank(A) = " << rank << std::endl;
  std::cout << "nullity(A) = " << nullity << std::endl;
  std::cout << "rank + nullity = " << (rank + nullity) << " (should be " << n << ")" << std::endl;

  // Null space basis from V (last columns corresponding to small singular values)
  Eigen::MatrixXd V = svd.matrixV(); // n x n
  Eigen::MatrixXd N = V.rightCols(nullity); // n x nullity

  // Verify A*N ≈ 0
  Eigen::MatrixXd R = A * N;
  std::cout << "Frobenius norm ||A*N||_F = " << R.norm() << std::endl;

  return 0;
}
