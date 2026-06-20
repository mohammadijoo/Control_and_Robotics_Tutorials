#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    MatrixXd A(3,3), B(3,1), C(1,3), D(1,1);
    A << 0, 1, 0,
         0, 0, 0,
         0, 0, -1;
    B << 0, 1, 0;
    C << 1, 0, 0;
    D << 0;

    int n = (int)A.rows();

    // Build O = [C; C A; ...; C A^(n-1)]
    MatrixXd O(n * C.rows(), n);
    MatrixXd Ak = MatrixXd::Identity(n,n);
    for (int k = 0; k < n; ++k) {
        O.block(k*C.rows(), 0, C.rows(), n) = C * Ak;
        Ak = Ak * A;
    }

    // Rank via SVD
    Eigen::JacobiSVD<MatrixXd> svd(O, Eigen::ComputeFullU | Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    double tol = 1e-10;
    int rank = 0;
    for (int i = 0; i < s.size(); ++i) if (s(i) > tol) rank++;

    std::cout << "O =\n" << O << "\n";
    std::cout << "rank(O) = " << rank << "\n";

    // Nullspace basis: columns of V corresponding to (near) zero singular values
    MatrixXd V = svd.matrixV();
    int null_dim = n - rank;
    MatrixXd Ny_basis = V.block(0, rank, n, null_dim);

    std::cout << "Ny basis (columns span ker(O)):\n" << Ny_basis << "\n";
    return 0;
}
      
