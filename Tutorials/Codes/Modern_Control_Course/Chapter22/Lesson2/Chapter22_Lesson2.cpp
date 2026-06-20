/*
Chapter22_Lesson2.cpp
Closed-Loop State Matrix and Mode Relocation

This standard C++17 example uses a from-scratch 2x2 SISO coefficient-matching
calculation. For larger control systems, use a matrix library such as Eigen
for linear algebra and a control library or custom pole-assignment routine.

Compile:
    g++ -std=c++17 Chapter22_Lesson2.cpp -o Chapter22_Lesson2

Run:
    ./Chapter22_Lesson2
*/

#include <array>
#include <complex>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

struct Matrix2 {
    double a11, a12, a21, a22;
};

struct Vector2 {
    double v1, v2;
};

Matrix2 closedLoopMatrix(const Matrix2& A, const Vector2& B, const Vector2& K) {
    // Acl = A - B K, with B 2x1 and K 1x2.
    return Matrix2{
        A.a11 - B.v1 * K.v1, A.a12 - B.v1 * K.v2,
        A.a21 - B.v2 * K.v1, A.a22 - B.v2 * K.v2
    };
}

double trace(const Matrix2& M) {
    return M.a11 + M.a22;
}

double determinant(const Matrix2& M) {
    return M.a11 * M.a22 - M.a12 * M.a21;
}

std::pair<double, double> characteristicCoefficients(const Matrix2& M) {
    // det(sI - M) = s^2 - tr(M)s + det(M) = s^2 + a1 s + a0.
    return {-trace(M), determinant(M)};
}

std::pair<std::complex<double>, std::complex<double>> eigenvalues2x2(const Matrix2& M) {
    const double tr = trace(M);
    const double det = determinant(M);
    const double disc = tr * tr - 4.0 * det;

    if (disc >= 0.0) {
        const double root = std::sqrt(disc);
        return {{0.5 * (tr + root), 0.0}, {0.5 * (tr - root), 0.0}};
    }

    const double root = std::sqrt(-disc);
    return {{0.5 * tr, 0.5 * root}, {0.5 * tr, -0.5 * root}};
}

Vector2 solve2x2(double m11, double m12, double m21, double m22,
                 double b1, double b2) {
    const double det = m11 * m22 - m12 * m21;
    if (std::abs(det) < 1e-12) {
        throw std::runtime_error("Singular coefficient-matching equations.");
    }

    return Vector2{
        ( b1 * m22 - m12 * b2) / det,
        ( m11 * b2 - b1 * m21) / det
    };
}

Vector2 secondOrderFeedbackByMatching(const Matrix2& A,
                                      const Vector2& B,
                                      double p1,
                                      double p2) {
    const double desiredA1 = -(p1 + p2);
    const double desiredA0 = p1 * p2;

    auto coeffsFor = [&](double k1, double k2) {
        Matrix2 Acl = closedLoopMatrix(A, B, Vector2{k1, k2});
        return characteristicCoefficients(Acl);
    };

    auto c00 = coeffsFor(0.0, 0.0);
    auto c10 = coeffsFor(1.0, 0.0);
    auto c01 = coeffsFor(0.0, 1.0);

    // [a1(k); a0(k)] = c00 + M [k1; k2]
    const double m11 = c10.first  - c00.first;
    const double m21 = c10.second - c00.second;
    const double m12 = c01.first  - c00.first;
    const double m22 = c01.second - c00.second;

    const double rhs1 = desiredA1 - c00.first;
    const double rhs2 = desiredA0 - c00.second;

    return solve2x2(m11, m12, m21, m22, rhs1, rhs2);
}

int main() {
    Matrix2 A{0.0, 1.0, -2.0, -0.4};
    Vector2 B{0.0, 1.0};

    auto openModes = eigenvalues2x2(A);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Open-loop modes:\n";
    std::cout << "lambda1 = " << openModes.first << "\n";
    std::cout << "lambda2 = " << openModes.second << "\n\n";

    const double p1 = -2.0;
    const double p2 = -3.0;
    Vector2 K = secondOrderFeedbackByMatching(A, B, p1, p2);
    Matrix2 Acl = closedLoopMatrix(A, B, K);

    auto closedModes = eigenvalues2x2(Acl);
    auto coeffs = characteristicCoefficients(Acl);

    std::cout << "K = [" << K.v1 << ", " << K.v2 << "]\n";
    std::cout << "Acl = [["
              << Acl.a11 << ", " << Acl.a12 << "], ["
              << Acl.a21 << ", " << Acl.a22 << "]]\n";
    std::cout << "Closed-loop modes:\n";
    std::cout << "lambda1 = " << closedModes.first << "\n";
    std::cout << "lambda2 = " << closedModes.second << "\n";
    std::cout << "Characteristic polynomial: s^2 + "
              << coeffs.first << " s + " << coeffs.second << "\n";

    return 0;
}
