#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd A_rolling_disk(const VectorXd& q, double R) {
    // q = [x, y, theta, phi]
    double theta = q(2);
    MatrixXd A(2, 4);
    A.setZero();

    // Constraint 1: xdot*cos(theta) + ydot*sin(theta) - R*phidot = 0
    A(0, 0) = std::cos(theta);
    A(0, 1) = std::sin(theta);
    A(0, 2) = 0.0;
    A(0, 3) = -R;

    // Constraint 2: xdot*sin(theta) - ydot*cos(theta) = 0
    A(1, 0) = std::sin(theta);
    A(1, 1) = -std::cos(theta);
    A(1, 2) = 0.0;
    A(1, 3) = 0.0;

    return A;
}

int main() {
    double R = 0.1;
    VectorXd q(4);
    q << 0.0, 0.0, 0.0, 0.0;

    MatrixXd A = A_rolling_disk(q, R);

    Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullV);
    const auto& S = svd.singularValues();
    MatrixXd V = svd.matrixV();

    int rank = 0;
    double tol = 1e-9;
    for (int i = 0; i < S.size(); ++i) {
        if (S(i) > tol) {
            rank++;
        }
    }
    int null_dim = V.cols() - rank;
    MatrixXd basis = V.block(0, rank, V.rows(), null_dim);

    std::cout << "rank(A)   = " << rank << std::endl;
    std::cout << "null_dim   = " << null_dim << std::endl;
    std::cout << "Nullspace basis:\n" << basis << std::endl;

    return 0;
}
      
