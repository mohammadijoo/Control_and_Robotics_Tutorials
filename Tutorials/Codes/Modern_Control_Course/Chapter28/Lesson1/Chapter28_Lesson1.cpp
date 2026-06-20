/*
Chapter28_Lesson1.cpp

Quadratic forms in state and input variables.

Dependency:
    Eigen 3
Compile example:
    g++ -std=c++17 Chapter28_Lesson1.cpp -I /path/to/eigen -O2 -o Chapter28_Lesson1
*/

#include <Eigen/Dense>
#include <iostream>
#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd symmetrize(const MatrixXd& M) {
    return 0.5 * (M + M.transpose());
}

std::string definiteness(const MatrixXd& M, double tol = 1e-10) {
    Eigen::SelfAdjointEigenSolver<MatrixXd> solver(symmetrize(M));
    VectorXd eigs = solver.eigenvalues();

    bool all_pos = true, all_nonneg = true, all_neg = true, all_nonpos = true;
    for (int i = 0; i < eigs.size(); ++i) {
        all_pos = all_pos && (eigs(i) > tol);
        all_nonneg = all_nonneg && (eigs(i) >= -tol);
        all_neg = all_neg && (eigs(i) < -tol);
        all_nonpos = all_nonpos && (eigs(i) <= tol);
    }

    if (all_pos) return "positive definite";
    if (all_nonneg) return "positive semidefinite";
    if (all_neg) return "negative definite";
    if (all_nonpos) return "negative semidefinite";
    return "indefinite";
}

double quadraticForm(const VectorXd& z, const MatrixXd& M) {
    return (z.transpose() * M * z)(0, 0);
}

double stageCost(const VectorXd& x, const VectorXd& u,
                 const MatrixXd& Q, const MatrixXd& R, const MatrixXd& N) {
    double xQx = (x.transpose() * Q * x)(0, 0);
    double uRu = (u.transpose() * R * u)(0, 0);
    double cross = 2.0 * (x.transpose() * N * u)(0, 0);
    return xQx + cross + uRu;
}

int main() {
    MatrixXd Q(2, 2);
    Q << 10.0, 0.0,
         0.0,  1.0;

    MatrixXd R(1, 1);
    R << 0.25;

    MatrixXd N(2, 1);
    N << 0.0,
         0.15;

    VectorXd x(2);
    x << 0.7, -0.2;

    VectorXd u(1);
    u << 0.4;

    std::cout << "Q is " << definiteness(Q) << std::endl;
    std::cout << "R is " << definiteness(R) << std::endl;
    std::cout << "x^T Q x = " << quadraticForm(x, Q) << std::endl;
    std::cout << "stage cost = " << stageCost(x, u, Q, R, N) << std::endl;

    MatrixXd RinvNT = R.ldlt().solve(N.transpose());
    MatrixXd Schur = Q - N * RinvNT;

    std::cout << "R^{-1} N^T =" << std::endl << RinvNT << std::endl;
    std::cout << "Q - N R^{-1} N^T =" << std::endl << Schur << std::endl;
    std::cout << "Schur complement is " << definiteness(Schur) << std::endl;

    return 0;
}
