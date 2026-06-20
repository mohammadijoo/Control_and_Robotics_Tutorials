/*
Chapter9_Lesson5.cpp

Modern Control — Chapter 9, Lesson 5
Examples of Stable, Marginal, and Unstable State Matrices

This compact C++ program classifies 2x2 real continuous-time matrices
A for x_dot = A x using the closed-form eigenvalues of a 2x2 matrix.

Compile:
    g++ -std=c++17 Chapter9_Lesson5.cpp -o Chapter9_Lesson5
Run:
    ./Chapter9_Lesson5
*/

#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

struct Matrix2 {
    double a11, a12, a21, a22;
};

std::pair<std::complex<double>, std::complex<double>> eigenvalues2x2(const Matrix2& A) {
    double tr = A.a11 + A.a22;
    double det = A.a11 * A.a22 - A.a12 * A.a21;
    std::complex<double> disc = std::complex<double>(tr * tr - 4.0 * det, 0.0);
    std::complex<double> root = std::sqrt(disc);
    return {
        0.5 * (std::complex<double>(tr, 0.0) + root),
        0.5 * (std::complex<double>(tr, 0.0) - root)
    };
}

double determinant(const Matrix2& A) {
    return A.a11 * A.a22 - A.a12 * A.a21;
}

int rank2x2(const Matrix2& A, double tol = 1e-10) {
    if (std::fabs(A.a11) < tol && std::fabs(A.a12) < tol &&
        std::fabs(A.a21) < tol && std::fabs(A.a22) < tol) {
        return 0;
    }
    if (std::fabs(determinant(A)) > tol) {
        return 2;
    }
    return 1;
}

std::string classify2x2(const Matrix2& A, double tol = 1e-9) {
    auto [lambda1, lambda2] = eigenvalues2x2(A);
    double max_real = std::max(lambda1.real(), lambda2.real());

    if (max_real < -tol) {
        return "asymptotically stable";
    }
    if (max_real > tol) {
        return "unstable";
    }

    // Boundary case for 2x2:
    // repeated eigenvalue on the imaginary axis is dangerous if it is defective.
    if (std::abs(lambda1 - lambda2) < 1e-8 && std::fabs(lambda1.real()) < 1e-8) {
        Matrix2 shifted{
            A.a11 - lambda1.real(), A.a12,
            A.a21, A.a22 - lambda1.real()
        };
        int rank = rank2x2(shifted);
        int geometric_multiplicity = 2 - rank;
        if (geometric_multiplicity < 2) {
            return "unstable";
        }
    }

    return "marginally stable";
}

void report(const std::string& name, const Matrix2& A) {
    auto [lambda1, lambda2] = eigenvalues2x2(A);

    std::cout << "\n" << name << "\n";
    std::cout << std::string(name.size(), '-') << "\n";
    std::cout << "A = [[" << A.a11 << ", " << A.a12 << "], ["
              << A.a21 << ", " << A.a22 << "]]\n";
    std::cout << "lambda1 = " << lambda1 << "\n";
    std::cout << "lambda2 = " << lambda2 << "\n";
    std::cout << "classification = " << classify2x2(A) << "\n";
}

int main() {
    std::vector<std::pair<std::string, Matrix2>> examples = {
        {"Stable diagonal", {-2.0, 0.0, 0.0, -5.0}},
        {"Stable damped oscillator", {0.0, 1.0, -4.0, -2.0}},
        {"Stable but nonnormal", {-1.0, 50.0, 0.0, -2.0}},
        {"Marginal oscillator", {0.0, 1.0, -1.0, 0.0}},
        {"Marginal with semisimple zero", {0.0, 0.0, 0.0, -2.0}},
        {"Unstable positive eigenvalue", {1.0, 0.0, 0.0, -2.0}},
        {"Unstable defective zero", {0.0, 1.0, 0.0, 0.0}}
    };

    for (const auto& item : examples) {
        report(item.first, item.second);
    }

    return 0;
}
