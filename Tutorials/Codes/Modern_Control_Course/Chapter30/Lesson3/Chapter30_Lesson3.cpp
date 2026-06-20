/*
Chapter30_Lesson3.cpp
State-space analysis with Eigen: controllability, observability, Ackermann pole placement, RK4 simulation.

Build example:
    g++ -std=c++17 Chapter30_Lesson3.cpp -I /path/to/eigen -O2 -o Chapter30_Lesson3
*/

#include <Eigen/Dense>
#include <complex>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = A.rows();
    const int m = B.cols();
    MatrixXd C(n, n * m);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        C.block(0, k * m, n, m) = Ak * B;
        Ak = A * Ak;
    }
    return C;
}

MatrixXd observabilityMatrix(const MatrixXd& A, const MatrixXd& Cmat) {
    const int n = A.rows();
    const int p = Cmat.rows();
    MatrixXd O(n * p, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        O.block(k * p, 0, p, n) = Cmat * Ak;
        Ak = Ak * A;
    }
    return O;
}

VectorXd characteristicFromDesiredPoles(const std::vector<std::complex<double>>& poles) {
    // Returns coefficients [a0, a1, ..., a_{n-1}] for s^n + a_{n-1}s^{n-1}+...+a0.
    std::vector<std::complex<double>> coeff{1.0};
    for (const auto& pole : poles) {
        std::vector<std::complex<double>> next(coeff.size() + 1, 0.0);
        for (std::size_t i = 0; i < coeff.size(); ++i) {
            next[i] += -pole * coeff[i];
            next[i + 1] += coeff[i];
        }
        coeff = next;
    }
    VectorXd realCoeff(static_cast<int>(poles.size()));
    for (int i = 0; i < static_cast<int>(poles.size()); ++i) {
        realCoeff(i) = coeff[i].real();
    }
    return realCoeff;
}

MatrixXd matrixPolynomial(const MatrixXd& A, const VectorXd& coeff) {
    // phi(A) = A^n + a_{n-1}A^{n-1}+...+a0 I, with coeff=[a0,...,a_{n-1}].
    const int n = A.rows();
    MatrixXd phi = MatrixXd::Zero(n, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int i = 0; i < n; ++i) {
        phi += coeff(i) * Ak;
        Ak = A * Ak;
    }
    phi += Ak;
    return phi;
}

MatrixXd ackermannGainSISO(const MatrixXd& A, const MatrixXd& B, const std::vector<std::complex<double>>& poles) {
    const int n = A.rows();
    MatrixXd Ctrb = controllabilityMatrix(A, B);
    Eigen::FullPivLU<MatrixXd> lu(Ctrb);
    if (lu.rank() != n) {
        throw std::runtime_error("System is not controllable; Ackermann formula is not valid.");
    }
    VectorXd coeff = characteristicFromDesiredPoles(poles);
    MatrixXd phiA = matrixPolynomial(A, coeff);
    MatrixXd eT = MatrixXd::Zero(1, n);
    eT(0, n - 1) = 1.0;
    return eT * Ctrb.inverse() * phiA;
}

VectorXd fClosedLoop(const MatrixXd& A, const MatrixXd& B, const MatrixXd& K, const VectorXd& x) {
    return (A - B * K) * x;
}

VectorXd rk4Step(const MatrixXd& A, const MatrixXd& B, const MatrixXd& K, const VectorXd& x, double h) {
    VectorXd k1 = fClosedLoop(A, B, K, x);
    VectorXd k2 = fClosedLoop(A, B, K, x + 0.5 * h * k1);
    VectorXd k3 = fClosedLoop(A, B, K, x + 0.5 * h * k2);
    VectorXd k4 = fClosedLoop(A, B, K, x + h * k3);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

int main() {
    MatrixXd A(2, 2), B(2, 1), C(1, 2);
    A << 0.0, 1.0,
        -2.0, -0.35;
    B << 0.0,
        1.0;
    C << 1.0, 0.0;

    MatrixXd Ctrb = controllabilityMatrix(A, B);
    MatrixXd Obsv = observabilityMatrix(A, C);
    std::cout << "Controllability matrix:\n" << Ctrb << "\nrank = " << Eigen::FullPivLU<MatrixXd>(Ctrb).rank() << "\n\n";
    std::cout << "Observability matrix:\n" << Obsv << "\nrank = " << Eigen::FullPivLU<MatrixXd>(Obsv).rank() << "\n\n";

    std::vector<std::complex<double>> poles{{-2.0, 1.5}, {-2.0, -1.5}};
    MatrixXd K = ackermannGainSISO(A, B, poles);
    std::cout << "Ackermann K = " << K << "\n";
    std::cout << "A - B*K:\n" << A - B * K << "\n\n";

    VectorXd x(2);
    x << 1.0, 0.0;
    double h = 0.01;
    for (int i = 0; i <= 500; ++i) {
        if (i % 100 == 0) {
            std::cout << "t=" << i * h << ", x=[" << x.transpose() << "]\n";
        }
        x = rk4Step(A, B, K, x, h);
    }
    return 0;
}
