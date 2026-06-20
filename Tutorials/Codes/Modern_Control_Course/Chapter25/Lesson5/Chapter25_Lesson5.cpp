// Chapter25_Lesson5.cpp
/*
Structural Constraints: Limited Actuators and Sparse Feedback

C++ implementation using Eigen:
    g++ -std=c++17 Chapter25_Lesson5.cpp -I /path/to/eigen -O2 -o Chapter25_Lesson5

Eigen is widely used in modern-control prototyping for matrix operations.
This example computes controllability rank and compares dense/sparse gains.
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = static_cast<int>(A.rows());
    const int m = static_cast<int>(B.cols());
    MatrixXd C(n, n * m);
    MatrixXd Ap = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        C.block(0, k * m, n, m) = Ap * B;
        Ap = Ap * A;
    }
    return C;
}

int numericalRank(const MatrixXd& M, double tol = 1e-9) {
    Eigen::JacobiSVD<MatrixXd> svd(M);
    const auto& s = svd.singularValues();
    int r = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) {
            ++r;
        }
    }
    return r;
}

MatrixXd applyMask(const MatrixXd& K, const MatrixXd& mask) {
    return K.cwiseProduct(mask);
}

VectorXd eulerSimulate(const MatrixXd& Acl, const VectorXd& x0, double dt, int steps) {
    VectorXd x = x0;
    for (int k = 0; k < steps; ++k) {
        x = x + dt * (Acl * x);
    }
    return x;
}

void printEigenvalues(const MatrixXd& Acl, const std::string& label) {
    Eigen::EigenSolver<MatrixXd> solver(Acl);
    std::cout << label << "\n";
    for (int i = 0; i < solver.eigenvalues().size(); ++i) {
        std::cout << "  " << solver.eigenvalues()(i) << "\n";
    }
}

int main() {
    MatrixXd A(4, 4);
    A << 0.0,  1.0,  0.0,  0.0,
        -2.0, -0.25, 0.7,  0.0,
         0.0,  0.0,  0.0,  1.0,
         0.6,  0.0, -1.5, -0.20;

    MatrixXd B(4, 2);
    B << 0.0, 0.0,
         1.0, 0.0,
         0.0, 0.0,
         0.0, 1.0;

    MatrixXd C = controllabilityMatrix(A, B);
    std::cout << "rank(C) = " << numericalRank(C) << " / " << A.rows() << "\n";

    // A representative dense stabilizing gain from an external design step.
    MatrixXd Kdense(2, 4);
    Kdense << 3.0, 2.0, 0.9, 0.4,
              0.6, 0.3, 2.6, 1.8;

    MatrixXd mask(2, 4);
    mask << 1.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.0;

    MatrixXd Ksparse = applyMask(Kdense, mask);

    std::cout << "\nDense K:\n" << Kdense << "\n";
    std::cout << "\nSparse K:\n" << Ksparse << "\n";

    MatrixXd AclDense = A - B * Kdense;
    MatrixXd AclSparse = A - B * Ksparse;

    printEigenvalues(A, "\nOpen-loop eigenvalues:");
    printEigenvalues(AclDense, "\nDense closed-loop eigenvalues:");
    printEigenvalues(AclSparse, "\nSparse closed-loop eigenvalues:");

    VectorXd x0(4);
    x0 << 1.0, 0.0, -0.7, 0.2;
    VectorXd xfDense = eulerSimulate(AclDense, x0, 0.001, 10000);
    VectorXd xfSparse = eulerSimulate(AclSparse, x0, 0.001, 10000);

    std::cout << "\nFinal dense state:\n" << xfDense.transpose() << "\n";
    std::cout << "Final sparse state:\n" << xfSparse.transpose() << "\n";

    return 0;
}
