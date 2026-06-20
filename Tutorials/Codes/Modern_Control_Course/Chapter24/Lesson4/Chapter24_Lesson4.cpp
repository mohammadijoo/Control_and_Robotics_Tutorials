// Chapter24_Lesson4.cpp
// Partial Pole Placement and Restricted Eigenstructure
// Dependency: Eigen 3
// Build example: g++ -std=c++17 Chapter24_Lesson4.cpp -I /path/to/eigen -O2 -o Chapter24_Lesson4

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    const int n = 4;
    const int m = 2;

    MatrixXd A = MatrixXd::Zero(n, n);
    A(0,0) = 0.2;
    A(1,1) = 0.6;
    A(2,2) = -0.5;
    A(3,3) = -1.0;

    MatrixXd B(n, m);
    B << 1.0, 0.0,
         0.3, 1.0,
         0.2, 0.4,
         0.1, 0.2;

    std::vector<double> lambdas = {-2.0, -3.0};
    std::vector<VectorXd> gList;
    VectorXd g1(m), g2(m);
    g1 << 1.0, 0.0;
    g2 << 0.0, 1.0;
    gList.push_back(g1);
    gList.push_back(g2);

    MatrixXd V(n, 2);
    MatrixXd G(m, 2);
    for (int i = 0; i < 2; ++i) {
        MatrixXd M = A - lambdas[i] * MatrixXd::Identity(n, n);
        VectorXd v = M.fullPivLu().solve(B * gList[i]);
        V.col(i) = v;
        G.col(i) = gList[i];
    }

    MatrixXd Nkeep = MatrixXd::Zero(n, 2);
    Nkeep(2,0) = 1.0;
    Nkeep(3,1) = 1.0;

    MatrixXd X(n, n);
    X << V, Nkeep;

    MatrixXd Y = MatrixXd::Zero(m, n);
    Y.block(0, 0, m, 2) = G;

    MatrixXd K = Y * X.inverse();
    MatrixXd Acl = A - B * K;

    Eigen::EigenSolver<MatrixXd> solver(Acl);

    std::cout << "K =\n" << K << "\n\n";
    std::cout << "Closed-loop eigenvalues =\n" << solver.eigenvalues() << "\n\n";
    std::cout << "Assigned-mode residual norm = "
              << (Acl * V - V * (VectorXd(2) << lambdas[0], lambdas[1]).finished().asDiagonal()).norm()
              << "\n";
    std::cout << "Preservation residual ||K Nkeep|| = " << (K * Nkeep).norm() << "\n\n";

    // Restricted eigenstructure: require x3 = x4 for lambda = -2.
    MatrixXd H(1, n);
    H << 0.0, 0.0, 1.0, -1.0;
    double lam = -2.0;
    MatrixXd M = (A - lam * MatrixXd::Identity(n, n)).inverse() * B;
    MatrixXd S = H * M;

    // For a 1-by-2 row S = [a b], a null vector is [b, -a]^T.
    VectorXd grest(m);
    grest << S(0,1), -S(0,0);
    VectorXd vrest = M * grest;

    std::cout << "S = H(A-lambda I)^(-1)B =\n" << S << "\n";
    std::cout << "Restricted g =\n" << grest << "\n";
    std::cout << "Restricted v =\n" << vrest << "\n";
    std::cout << "H v =\n" << H * vrest << "\n";

    return 0;
}
