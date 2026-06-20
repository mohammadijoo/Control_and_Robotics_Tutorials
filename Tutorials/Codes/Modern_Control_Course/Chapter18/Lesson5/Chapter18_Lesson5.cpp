/*
Chapter18_Lesson5.cpp
Interpretation of Jordan form in control applications using Eigen.

Build example on Linux/macOS:
    g++ -std=c++17 Chapter18_Lesson5.cpp -I /usr/include/eigen3 -O2 -o Chapter18_Lesson5

Build example on Windows with MinGW and Eigen:
    g++ -std=c++17 Chapter18_Lesson5.cpp -I C:\\Eigen -O2 -o Chapter18_Lesson5.exe
*/

#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using Eigen::FullPivLU;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd jordanBlock(double lambda, int n) {
    MatrixXd J = lambda * MatrixXd::Identity(n, n);
    for (int i = 0; i < n - 1; ++i) {
        J(i, i + 1) = 1.0;
    }
    return J;
}

double factorial(int k) {
    double f = 1.0;
    for (int i = 2; i <= k; ++i) f *= static_cast<double>(i);
    return f;
}

MatrixXd jordanBlockExponential(double lambda, int n, double t) {
    MatrixXd E = MatrixXd::Zero(n, n);
    const double scale = std::exp(lambda * t);
    for (int i = 0; i < n; ++i) {
        for (int j = i; j < n; ++j) {
            const int power = j - i;
            E(i, j) = scale * std::pow(t, power) / factorial(power);
        }
    }
    return E;
}

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = static_cast<int>(A.rows());
    const int m = static_cast<int>(B.cols());
    MatrixXd Ctrb(n, n * m);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        Ctrb.block(0, k * m, n, m) = Ak * B;
        Ak = A * Ak;
    }
    return Ctrb;
}

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& C) {
    const int n = static_cast<int>(A.rows());
    const int p = static_cast<int>(C.rows());
    MatrixXd Obsv(n * p, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        Obsv.block(k * p, 0, p, n) = C * Ak;
        Ak = Ak * A;
    }
    return Obsv;
}

int rankOf(const MatrixXd& M) {
    FullPivLU<MatrixXd> lu(M);
    lu.setThreshold(1e-10);
    return static_cast<int>(lu.rank());
}

int main() {
    const double lambda = -1.0;
    MatrixXd A = jordanBlock(lambda, 3);
    std::cout << "A = Jordan block J_3(-1):\n" << A << "\n\n";

    MatrixXd E = jordanBlockExponential(lambda, 3, 2.0);
    std::cout << "Closed-form exp(A t) at t = 2:\n" << E << "\n\n";

    MatrixXd Bgood(3, 1);
    Bgood << 0.0, 0.0, 1.0;
    MatrixXd Bbad(3, 1);
    Bbad << 1.0, 0.0, 0.0;

    MatrixXd WcGood = controllabilityMatrix(A, Bgood);
    MatrixXd WcBad = controllabilityMatrix(A, Bbad);

    std::cout << "Ctrb(A, Bgood):\n" << WcGood << "\nrank = " << rankOf(WcGood) << "\n\n";
    std::cout << "Ctrb(A, Bbad):\n" << WcBad << "\nrank = " << rankOf(WcBad) << "\n\n";

    MatrixXd Cgood(1, 3);
    Cgood << 1.0, 0.0, 0.0;
    MatrixXd Cbad(1, 3);
    Cbad << 0.0, 0.0, 1.0;

    MatrixXd WoGood = observabilityMatrix(A, Cgood);
    MatrixXd WoBad = observabilityMatrix(A, Cbad);

    std::cout << "Obsv(A, Cgood):\n" << WoGood << "\nrank = " << rankOf(WoGood) << "\n\n";
    std::cout << "Obsv(A, Cbad):\n" << WoBad << "\nrank = " << rankOf(WoBad) << "\n\n";

    return 0;
}
