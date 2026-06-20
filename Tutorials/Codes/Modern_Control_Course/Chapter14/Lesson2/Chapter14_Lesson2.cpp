/*
Chapter14_Lesson2.cpp
PBH Test for Observability using Eigen.

Compile example:
    g++ -std=c++17 Chapter14_Lesson2.cpp -I /path/to/eigen -O2 -o pbh_obs
*/

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <complex>
#include <iostream>
#include <vector>

using Complex = std::complex<double>;
using MatrixXd = Eigen::MatrixXd;
using MatrixXcd = Eigen::MatrixXcd;

int complexRank(const MatrixXcd& M, double tol = 1e-9) {
    Eigen::FullPivLU<MatrixXcd> lu(M);
    lu.setThreshold(tol);
    return lu.rank();
}

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& C) {
    const int n = static_cast<int>(A.rows());
    const int p = static_cast<int>(C.rows());
    MatrixXd O(p * n, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        O.block(k * p, 0, p, n) = C * Ak;
        Ak = Ak * A;
    }
    return O;
}

bool pbhObservable(const MatrixXd& A, const MatrixXd& C, double tol = 1e-9) {
    const int n = static_cast<int>(A.rows());
    const int p = static_cast<int>(C.rows());

    Eigen::EigenSolver<MatrixXd> solver(A);
    const auto eigenvalues = solver.eigenvalues();

    bool observable = true;
    for (int k = 0; k < eigenvalues.size(); ++k) {
        Complex lambda = eigenvalues(k);
        MatrixXcd M(n + p, n);
        M.topRows(n) = lambda * MatrixXcd::Identity(n, n) - A.cast<Complex>();
        M.bottomRows(p) = C.cast<Complex>();

        int r = complexRank(M, tol);
        bool passed = (r == n);
        observable = observable && passed;

        std::cout << "lambda = " << lambda
                  << ", rank = " << r
                  << ", PBH mode passed = " << std::boolalpha << passed
                  << "\n";
    }
    return observable;
}

int main() {
    MatrixXd A1(2, 2);
    A1 << 0.0, 1.0,
         -2.0, -3.0;
    MatrixXd C1(1, 2);
    C1 << 1.0, 0.0;

    MatrixXd A2(2, 2);
    A2 << -1.0, 0.0,
           0.0, -2.0;
    MatrixXd C2(1, 2);
    C2 << 1.0, 0.0;

    std::vector<std::pair<MatrixXd, MatrixXd>> examples = {{A1, C1}, {A2, C2}};

    for (std::size_t i = 0; i < examples.size(); ++i) {
        std::cout << "================ Example " << i + 1 << " ================\n";
        const MatrixXd& A = examples[i].first;
        const MatrixXd& C = examples[i].second;
        MatrixXd O = observabilityMatrix(A, C);
        Eigen::FullPivLU<MatrixXd> lu(O);
        std::cout << "Rank of observability matrix = " << lu.rank() << "\n";
        bool ok = pbhObservable(A, C);
        std::cout << "Observable by PBH = " << std::boolalpha << ok << "\n";
    }

    return 0;
}
