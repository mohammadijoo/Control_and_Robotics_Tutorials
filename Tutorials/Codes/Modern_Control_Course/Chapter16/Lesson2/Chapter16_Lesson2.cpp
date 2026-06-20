/*
Chapter16_Lesson2.cpp

Construction of Controllable Canonical Form (CCF) from SISO transfer-function data.

Compile:
    g++ -std=c++17 Chapter16_Lesson2.cpp -o Chapter16_Lesson2
*/

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using Matrix = std::vector<std::vector<double>>;

std::vector<double> trimLeadingZeros(const std::vector<double>& c, double tol = 1e-12) {
    if (c.empty()) {
        throw std::invalid_argument("Coefficient vector cannot be empty.");
    }
    std::size_t i = 0;
    while (i + 1 < c.size() && std::abs(c[i]) < tol) {
        ++i;
    }
    return std::vector<double>(c.begin() + static_cast<long>(i), c.end());
}

struct StateSpace {
    Matrix A;
    Matrix B;
    Matrix C;
    double D;
};

StateSpace controllableCanonicalForm(std::vector<double> num, std::vector<double> den) {
    num = trimLeadingZeros(num);
    den = trimLeadingZeros(den);

    if (std::abs(den[0]) < 1e-12) {
        throw std::invalid_argument("Leading denominator coefficient must be nonzero.");
    }

    const double leading = den[0];
    for (double& v : den) v /= leading;
    for (double& v : num) v /= leading;

    const int n = static_cast<int>(den.size()) - 1;
    if (n < 1) {
        throw std::invalid_argument("Denominator degree must be at least one.");
    }
    if (static_cast<int>(num.size()) > n + 1) {
        throw std::invalid_argument("Improper transfer function: numerator degree exceeds denominator degree.");
    }

    std::vector<double> numFull(n + 1, 0.0);
    const int offset = (n + 1) - static_cast<int>(num.size());
    for (int i = 0; i < static_cast<int>(num.size()); ++i) {
        numFull[offset + i] = num[i];
    }

    const double Dfeed = numFull[0];
    std::vector<double> rem(n + 1, 0.0);
    for (int i = 0; i <= n; ++i) {
        rem[i] = numFull[i] - Dfeed * den[i];
    }

    Matrix A(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n - 1; ++i) {
        A[i][i + 1] = 1.0;
    }
    // Last row: [-a_n, -a_(n-1), ..., -a_1]
    for (int j = 0; j < n; ++j) {
        A[n - 1][j] = -den[n - j];
    }

    Matrix B(n, std::vector<double>(1, 0.0));
    B[n - 1][0] = 1.0;

    Matrix C(1, std::vector<double>(n, 0.0));
    // rem[1], ..., rem[n] are coefficients of s^(n-1), ..., s^0.
    // C = [constant, coefficient of s, ..., coefficient of s^(n-1)].
    for (int j = 0; j < n; ++j) {
        C[0][j] = rem[n - j];
    }

    return {A, B, C, Dfeed};
}

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double v : row) {
            std::cout << std::setw(12) << std::setprecision(6) << std::fixed << v << " ";
        }
        std::cout << "\n";
    }
}

int main() {
    try {
        // G(s) = (2 s^2 + 5 s + 3)/(s^3 + 4 s^2 + 6 s + 8)
        std::vector<double> num{2, 5, 3};
        std::vector<double> den{1, 4, 6, 8};

        StateSpace ss = controllableCanonicalForm(num, den);

        printMatrix("A", ss.A);
        printMatrix("B", ss.B);
        printMatrix("C", ss.C);
        std::cout << "D = " << ss.D << "\n";
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
