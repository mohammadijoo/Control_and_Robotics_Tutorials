// Chapter13_Lesson3.cpp
// Detectability and Stable Unobservable Modes
//
// Dependency: Eigen 3
// Compile example:
//   g++ -std=c++17 Chapter13_Lesson3.cpp -I /path/to/eigen -O2 -o detectability
//
// Continuous time: all unobservable modes must satisfy Re(lambda) < 0.
// Discrete time: all unobservable modes must satisfy |lambda| < 1.

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <complex>
#include <vector>
#include <string>

using Matrix = Eigen::MatrixXd;
using VectorC = Eigen::VectorXcd;

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    const int n = static_cast<int>(A.rows());
    const int p = static_cast<int>(C.rows());

    Matrix O(p * n, n);
    Matrix Ak = Matrix::Identity(n, n);

    for (int k = 0; k < n; ++k) {
        O.block(k * p, 0, p, n) = C * Ak;
        Ak = Ak * A;
    }
    return O;
}

Matrix nullspace(const Matrix& M, double tol = 1e-10) {
    Eigen::JacobiSVD<Matrix> svd(M, Eigen::ComputeFullV);
    const auto& s = svd.singularValues();
    int rank = 0;
    double scale = (s.size() == 0) ? 0.0 : s(0);
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol * std::max(M.rows(), M.cols()) * scale) {
            ++rank;
        }
    }

    Matrix V = svd.matrixV();
    return V.rightCols(M.cols() - rank);
}

VectorC unobservableModes(const Matrix& A, const Matrix& C, double tol = 1e-10) {
    Matrix O = observabilityMatrix(A, C);
    Matrix N = nullspace(O, tol);

    if (N.cols() == 0) {
        return VectorC(0);
    }

    Matrix Ahidden = N.transpose() * A * N;
    Eigen::EigenSolver<Matrix> es(Ahidden);
    return es.eigenvalues();
}

bool isDetectable(const Matrix& A, const Matrix& C,
                  const std::string& system = "continuous",
                  double tol = 1e-9) {
    VectorC hidden = unobservableModes(A, C, tol);

    if (hidden.size() == 0) {
        return true;
    }

    if (system == "continuous") {
        for (int i = 0; i < hidden.size(); ++i) {
            if (hidden(i).real() >= -tol) {
                return false;
            }
        }
        return true;
    }

    if (system == "discrete") {
        for (int i = 0; i < hidden.size(); ++i) {
            if (std::abs(hidden(i)) >= 1.0 - tol) {
                return false;
            }
        }
        return true;
    }

    throw std::runtime_error("system must be continuous or discrete");
}

void report(const Matrix& A, const Matrix& C,
            const std::string& name,
            const std::string& system = "continuous") {
    Matrix O = observabilityMatrix(A, C);
    VectorC hidden = unobservableModes(A, C);

    std::cout << "\n" << name << "\n";
    std::cout << std::string(name.size(), '-') << "\n";
    std::cout << "A =\n" << A << "\n";
    std::cout << "C =\n" << C << "\n";
    std::cout << "rank(O) = "
              << Eigen::FullPivLU<Matrix>(O).rank()
              << " of n = " << A.rows() << "\n";

    std::cout << "unobservable modes = ";
    for (int i = 0; i < hidden.size(); ++i) {
        std::cout << hidden(i) << " ";
    }
    std::cout << "\n";

    std::cout << "detectable = "
              << (isDetectable(A, C, system) ? "true" : "false")
              << "\n";
}

int main() {
    Matrix A1 = Matrix::Zero(3, 3);
    A1(0, 0) = -1.0;
    A1(1, 1) = 2.0;
    A1(2, 2) = -0.5;
    Matrix C1(1, 3);
    C1 << 0.0, 1.0, 0.0;
    report(A1, C1, "Continuous-time: detectable but not observable");

    Matrix A2 = Matrix::Zero(3, 3);
    A2(0, 0) = 1.0;
    A2(1, 1) = -2.0;
    A2(2, 2) = -0.5;
    Matrix C2(1, 3);
    C2 << 0.0, 1.0, 0.0;
    report(A2, C2, "Continuous-time: not detectable");

    Matrix A3 = Matrix::Zero(3, 3);
    A3(0, 0) = 0.3;
    A3(1, 1) = 1.2;
    A3(2, 2) = -0.7;
    Matrix C3(1, 3);
    C3 << 0.0, 1.0, 0.0;
    report(A3, C3, "Discrete-time: detectable but not observable", "discrete");

    return 0;
}
