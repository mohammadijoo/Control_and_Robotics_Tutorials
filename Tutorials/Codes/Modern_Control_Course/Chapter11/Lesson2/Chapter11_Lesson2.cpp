// Chapter11_Lesson2.cpp
// PBH controllability test using the Eigen C++ library.
// Compile example:
//   g++ -std=c++17 Chapter11_Lesson2.cpp -I /path/to/eigen -O2 -o pbh_test

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <complex>
#include <iostream>
#include <vector>

using Eigen::ComplexEigenSolver;
using Eigen::JacobiSVD;
using Eigen::MatrixXd;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;
using std::complex;
using std::cout;
using std::endl;
using std::vector;

int numericalRank(const MatrixXcd& M, double tol) {
    JacobiSVD<MatrixXcd> svd(M);
    auto s = svd.singularValues();
    int rank = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) {
            ++rank;
        }
    }
    return rank;
}

vector<complex<double>> uniqueEigenvalues(const VectorXcd& eig, double tol) {
    vector<complex<double>> values;
    for (int i = 0; i < eig.size(); ++i) {
        bool found = false;
        for (const auto& old : values) {
            if (std::abs(eig(i) - old) <= tol) {
                found = true;
                break;
            }
        }
        if (!found) {
            values.push_back(eig(i));
        }
    }
    return values;
}

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = A.rows();
    const int m = B.cols();
    MatrixXd C(n, n * m);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        C.block(0, k * m, n, m) = Ak * B;
        Ak = Ak * A;
    }
    return C;
}

bool pbhRankTest(const MatrixXd& A, const MatrixXd& B, double tol = 1e-9) {
    const int n = A.rows();
    const int m = B.cols();
    ComplexEigenSolver<MatrixXd> eig(A);
    vector<complex<double>> lambdas = uniqueEigenvalues(eig.eigenvalues(), 1e-8);

    bool controllable = true;
    for (const auto& lambda : lambdas) {
        MatrixXcd M(n, n + m);
        M.block(0, 0, n, n) = lambda * MatrixXcd::Identity(n, n) - A.cast<complex<double>>();
        M.block(0, n, n, m) = B.cast<complex<double>>();
        int r = numericalRank(M, tol);
        cout << "lambda = " << lambda << ", rank([lambda I - A, B]) = " << r << "/" << n << endl;
        if (r != n) {
            controllable = false;
        }
    }
    return controllable;
}

int main() {
    MatrixXd A(3, 3);
    A << 0.0, 0.0, 0.0,
         0.0, -1.0, 0.0,
         0.0, 0.0, -2.0;

    MatrixXd B_bad(3, 1);
    B_bad << 0.0, 1.0, 1.0;

    MatrixXd B_good(3, 1);
    B_good << 1.0, 1.0, 1.0;

    cout << "Example 1: unactuated first mode" << endl;
    cout << "Kalman rank = " << numericalRank(controllabilityMatrix(A, B_bad).cast<complex<double>>(), 1e-9) << endl;
    cout << "PBH controllable? " << (pbhRankTest(A, B_bad) ? "yes" : "no") << "\n" << endl;

    cout << "Example 2: all modes actuated" << endl;
    cout << "Kalman rank = " << numericalRank(controllabilityMatrix(A, B_good).cast<complex<double>>(), 1e-9) << endl;
    cout << "PBH controllable? " << (pbhRankTest(A, B_good) ? "yes" : "no") << endl;

    return 0;
}
