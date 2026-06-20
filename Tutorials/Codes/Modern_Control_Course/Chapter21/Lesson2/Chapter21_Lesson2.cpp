/*
Chapter21_Lesson2.cpp
Computation of invariant zeros for square state-space systems using Eigen.

Requires Eigen 3:
    g++ -std=c++17 Chapter21_Lesson2.cpp -I /path/to/eigen -O2 -o zeros

For a square system, finite zeros are generalized eigenvalues of
    M v = lambda E v,
where
    E = [[I, 0], [0, 0]],     M = [[A, B], [-C, -D]].
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <complex>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXcd;
using std::complex;
using std::cout;
using std::endl;

struct Pencil {
    MatrixXd M;
    MatrixXd E;
};

Pencil rosenbrockPencil(const MatrixXd& A,
                        const MatrixXd& B,
                        const MatrixXd& C,
                        const MatrixXd& D) {
    const int n = A.rows();
    const int p = C.rows();
    const int m = B.cols();

    MatrixXd E = MatrixXd::Zero(n + p, n + m);
    E.block(0, 0, n, n) = MatrixXd::Identity(n, n);

    MatrixXd M(n + p, n + m);
    M.block(0, 0, n, n) = A;
    M.block(0, n, n, m) = B;
    M.block(n, 0, p, n) = -C;
    M.block(n, n, p, m) = -D;

    return {M, E};
}

std::vector<complex<double>> finiteInvariantZerosSquare(const MatrixXd& A,
                                                        const MatrixXd& B,
                                                        const MatrixXd& C,
                                                        const MatrixXd& D,
                                                        double tol = 1e-10) {
    if (D.rows() != D.cols()) {
        throw std::runtime_error("This example requires a square system: p == m.");
    }

    Pencil P = rosenbrockPencil(A, B, C, D);
    Eigen::GeneralizedEigenSolver<MatrixXd> ges(P.M, P.E);

    VectorXcd alpha = ges.alphas();
    VectorXcd beta = ges.betas();

    std::vector<complex<double>> zeros;
    for (int i = 0; i < alpha.size(); ++i) {
        if (std::abs(beta(i)) > tol) {
            zeros.push_back(alpha(i) / beta(i));
        }
    }
    return zeros;
}

VectorXcd zerosWhenDIsInvertible(const MatrixXd& A,
                                 const MatrixXd& B,
                                 const MatrixXd& C,
                                 const MatrixXd& D) {
    Eigen::FullPivLU<MatrixXd> lu(D);
    if (!lu.isInvertible()) {
        throw std::runtime_error("D is singular; use the Rosenbrock pencil.");
    }
    MatrixXd Az = A - B * D.inverse() * C;
    Eigen::EigenSolver<MatrixXd> es(Az);
    return es.eigenvalues();
}

int main() {
    MatrixXd A(2, 2);
    A << 0.0, 1.0,
        -2.0, -3.0;

    MatrixXd B(2, 1);
    B << 0.0,
         1.0;

    MatrixXd C(1, 2);
    C << 4.0, 1.0;

    MatrixXd D = MatrixXd::Zero(1, 1);

    auto zeros = finiteInvariantZerosSquare(A, B, C, D);
    cout << "Finite invariant zeros from Rosenbrock pencil:" << endl;
    for (const auto& z : zeros) {
        cout << z << endl;
    }

    MatrixXd D2(1, 1);
    D2 << 1.0;
    cout << "Zeros when D is invertible:" << endl;
    cout << zerosWhenDIsInvertible(A, B, C, D2) << endl;
    return 0;
}
