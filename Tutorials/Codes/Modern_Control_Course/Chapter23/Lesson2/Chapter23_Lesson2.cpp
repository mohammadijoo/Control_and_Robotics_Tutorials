/*
Chapter23_Lesson2.cpp
Pole Assignment via Controllable Canonical Form (CCF)

This example computes K_c by coefficient matching for a system already in CCF.
Compile:
    g++ -std=c++17 Chapter23_Lesson2.cpp -o Chapter23_Lesson2
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

Matrix companionMatrix(const Vector& aAscending) {
    const int n = static_cast<int>(aAscending.size());
    Matrix A(n, Vector(n, 0.0));
    for (int i = 0; i < n - 1; ++i) {
        A[i][i + 1] = 1.0;
    }
    for (int j = 0; j < n; ++j) {
        A[n - 1][j] = -aAscending[j];
    }
    return A;
}

Vector inputVector(int n) {
    Vector b(n, 0.0);
    b[n - 1] = 1.0;
    return b;
}

Vector polynomialFromRealRoots(const Vector& roots) {
    // Returns descending coefficients for product (s-root_i):
    // [1, alpha_{n-1}, ..., alpha_0]
    Vector coeff(1, 1.0);
    for (double r : roots) {
        Vector next(coeff.size() + 1, 0.0);
        for (std::size_t i = 0; i < coeff.size(); ++i) {
            next[i] += coeff[i];
            next[i + 1] += -r * coeff[i];
        }
        coeff = next;
    }
    return coeff;
}

Vector desiredCoefficientsAscending(const Vector& desiredPoles) {
    Vector desc = polynomialFromRealRoots(desiredPoles);
    const int n = static_cast<int>(desiredPoles.size());
    Vector asc(n);
    for (int i = 0; i < n; ++i) {
        asc[i] = desc[n - i];
    }
    return asc;
}

Vector ccfGain(const Vector& openCoeffsAscending, const Vector& desiredPoles) {
    Vector alpha = desiredCoefficientsAscending(desiredPoles);
    if (alpha.size() != openCoeffsAscending.size()) {
        throw std::runtime_error("System order and number of desired poles differ.");
    }
    Vector K(alpha.size());
    for (std::size_t i = 0; i < alpha.size(); ++i) {
        K[i] = alpha[i] - openCoeffsAscending[i];
    }
    return K;
}

Matrix closedLoopMatrix(const Matrix& A, const Vector& b, const Vector& K) {
    const int n = static_cast<int>(A.size());
    Matrix Acl = A;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            Acl[i][j] -= b[i] * K[j];
        }
    }
    return Acl;
}

void printMatrix(const Matrix& A, const std::string& name) {
    std::cout << name << " =\n";
    for (const auto& row : A) {
        for (double v : row) {
            std::cout << std::setw(12) << std::setprecision(6) << v << " ";
        }
        std::cout << "\n";
    }
}

void printVector(const Vector& v, const std::string& name) {
    std::cout << name << " = [ ";
    for (double x : v) {
        std::cout << std::setprecision(6) << x << " ";
    }
    std::cout << "]\n";
}

int main() {
    // p(s)=s^3+6s^2+11s+6; desired poles {-4,-5,-6}
    Vector a = {6.0, 11.0, 6.0};          // [a0,a1,a2]
    Vector desiredPoles = {-4.0, -5.0, -6.0};

    Matrix Ac = companionMatrix(a);
    Vector bc = inputVector(static_cast<int>(a.size()));
    Vector Kc = ccfGain(a, desiredPoles);
    Matrix Acl = closedLoopMatrix(Ac, bc, Kc);

    printMatrix(Ac, "A_c");
    printVector(bc, "b_c");
    printVector(Kc, "K_c");
    printMatrix(Acl, "A_c - b_c K_c");

    std::cout << "\nExpected closed-loop characteristic polynomial:\n";
    std::cout << "s^3 + 15 s^2 + 74 s + 120 = (s+4)(s+5)(s+6)\n";
    return 0;
}
