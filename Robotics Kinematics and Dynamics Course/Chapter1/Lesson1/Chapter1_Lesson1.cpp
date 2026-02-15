#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    MatrixXd A(2, 3);
    A << 1.0, 1.0, 0.0,
          0.0, 1.0, 1.0;

    std::cout << "A =\n" << A << std::endl;

    // Compute SVD for rank and null space
    Eigen::JacobiSVD<MatrixXd> svd(
        A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto S = svd.singularValues();
    auto V = svd.matrixV();

    double tol = 1e-10;
    int n = A.cols();
    int rank = 0;
    for (int i = 0; i < S.size(); ++i) {
        if (S(i) > tol) {
            rank++;
        }
    }
    std::cout << "rank(A) = " << rank << std::endl;

    // Columns of V corresponding to zero singular values span the null space
    std::cout << "Approximate null-space basis:\n";
    for (int i = 0; i < n; ++i) {
        if (S(i) <= tol) {
            std::cout << "basis vector " << i << ":\n"
                      << V.col(i) << "\n";
        }
    }
    return 0;
}
      
