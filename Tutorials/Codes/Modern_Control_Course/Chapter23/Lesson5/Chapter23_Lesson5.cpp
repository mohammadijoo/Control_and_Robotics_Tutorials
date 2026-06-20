// Chapter23_Lesson5.cpp
// Numerical Sensitivity and Conditioning in SISO Pole Placement
//
// Dependency: Eigen 3
// Compile example:
//   g++ -std=c++17 Chapter23_Lesson5.cpp -I /path/to/eigen -O2 -o Chapter23_Lesson5
//
// This program implements Ackermann's formula for a 3rd-order SISO system
// and prints controllability conditioning and closed-loop poles.

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>
#include <complex>
#include <iomanip>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    int n = A.rows();
    Matrix C(n, n);
    Matrix Ak = Matrix::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        C.col(k) = Ak * B;
        Ak = A * Ak;
    }
    return C;
}

Matrix polynomialMatrix(const Matrix& A, const std::vector<double>& coeffs) {
    int n = A.rows();
    Matrix P = Matrix::Zero(n, n);
    Matrix I = Matrix::Identity(n, n);
    for (double c : coeffs) {
        P = P * A + c * I;
    }
    return P;
}

std::vector<double> polyFromRoots3(double r1, double r2, double r3) {
    // (s-r1)(s-r2)(s-r3) = s^3 + a2 s^2 + a1 s + a0
    double a2 = -(r1 + r2 + r3);
    double a1 = r1*r2 + r1*r3 + r2*r3;
    double a0 = -r1*r2*r3;
    return {1.0, a2, a1, a0};
}

Matrix ackermannGain(const Matrix& A, const Matrix& B, const std::vector<double>& coeffs) {
    int n = A.rows();
    Matrix C = controllabilityMatrix(A, B);
    Matrix pA = polynomialMatrix(A, coeffs);
    Matrix eT = Matrix::Zero(1, n);
    eT(0, n - 1) = 1.0;
    Matrix K = eT * C.inverse() * pA;
    return K;
}

double conditionNumber2(const Matrix& M) {
    Eigen::JacobiSVD<Matrix> svd(M);
    double smax = svd.singularValues()(0);
    double smin = svd.singularValues()(svd.singularValues().size() - 1);
    return smax / smin;
}

void printEigenvalues(const Matrix& M) {
    Eigen::EigenSolver<Matrix> solver(M);
    std::cout << solver.eigenvalues() << "\n";
}

int main() {
    std::cout << std::setprecision(8);

    Matrix A(3, 3);
    A << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
        -2.0,-3.0,-4.0;

    Matrix B(3, 1);
    B << 0.0, 0.0, 1.0;

    auto coeffs = polyFromRoots3(-4.0, -5.0, -6.0);
    Matrix C = controllabilityMatrix(A, B);
    Matrix K = ackermannGain(A, B, coeffs);
    Matrix M = A - B * K;

    std::cout << "Original coordinates\n";
    std::cout << "--------------------\n";
    std::cout << "Desired polynomial coefficients: ";
    for (double c : coeffs) std::cout << c << " ";
    std::cout << "\n";
    std::cout << "cond(C) = " << conditionNumber2(C) << "\n";
    std::cout << "K = " << K << "\n";
    std::cout << "Closed-loop eigenvalues:\n";
    printEigenvalues(M);

    Matrix Tbad = Matrix::Zero(3, 3);
    Tbad(0,0) = 1e-3;
    Tbad(1,1) = 1.0;
    Tbad(2,2) = 1e3;

    Matrix Abad = Tbad.inverse() * A * Tbad;
    Matrix Bbad = Tbad.inverse() * B;
    Matrix Cbad = controllabilityMatrix(Abad, Bbad);
    Matrix Kz = ackermannGain(Abad, Bbad, coeffs);
    Matrix Kx = Kz * Tbad.inverse();

    std::cout << "\nBadly scaled design coordinates\n";
    std::cout << "-------------------------------\n";
    std::cout << "cond(C_bad) = " << conditionNumber2(Cbad) << "\n";
    std::cout << "K_z = " << Kz << "\n";
    std::cout << "Equivalent K_x = " << Kx << "\n";
    std::cout << "Closed-loop eigenvalues:\n";
    printEigenvalues(A - B * Kx);

    return 0;
}
