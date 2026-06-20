/*
Chapter8_Lesson2.cpp
Modern Control — Chapter 8, Lesson 2
Semigroup Property and Inverse of the State Transition Matrix Phi(t)

Compile:
  g++ -std=c++17 Chapter8_Lesson2.cpp -O2 -o Chapter8_Lesson2
*/

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Matrix2 = std::array<std::array<double, 2>, 2>;
using Vector2 = std::array<double, 2>;

Matrix2 zero2() {
    return {{{0.0, 0.0}, {0.0, 0.0}}};
}

Matrix2 identity2() {
    return {{{1.0, 0.0}, {0.0, 1.0}}};
}

Matrix2 add(const Matrix2& A, const Matrix2& B) {
    Matrix2 C = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

Matrix2 scale(const Matrix2& A, double c) {
    Matrix2 B = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            B[i][j] = c * A[i][j];
    return B;
}

Matrix2 multiply(const Matrix2& A, const Matrix2& B) {
    Matrix2 C = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Vector2 multiply(const Matrix2& A, const Vector2& x) {
    return {A[0][0] * x[0] + A[0][1] * x[1],
            A[1][0] * x[0] + A[1][1] * x[1]};
}

double frobeniusNorm(const Matrix2& A) {
    double sum = 0.0;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            sum += A[i][j] * A[i][j];
    return std::sqrt(sum);
}

double vectorNorm(const Vector2& x) {
    return std::sqrt(x[0] * x[0] + x[1] * x[1]);
}

Matrix2 subtract(const Matrix2& A, const Matrix2& B) {
    return add(A, scale(B, -1.0));
}

Vector2 subtract(const Vector2& a, const Vector2& b) {
    return {a[0] - b[0], a[1] - b[1]};
}

Matrix2 inverse2(const Matrix2& A) {
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    return {{{ A[1][1] / det, -A[0][1] / det},
             {-A[1][0] / det,  A[0][0] / det}}};
}

Matrix2 matrixExpTaylor2(const Matrix2& A, double t) {
    // Scaling-and-squaring with Taylor series.
    // For teaching purposes: robust enough for small 2x2 examples.
    Matrix2 M = scale(A, t);
    double normM = frobeniusNorm(M);
    int scalingPower = std::max(0, static_cast<int>(std::ceil(std::log2(normM + 1e-12))));
    double divisor = std::pow(2.0, scalingPower);
    Matrix2 B = scale(M, 1.0 / divisor);

    Matrix2 result = identity2();
    Matrix2 term = identity2();
    const int terms = 40;
    for (int k = 1; k <= terms; ++k) {
        term = multiply(term, scale(B, 1.0 / static_cast<double>(k)));
        result = add(result, term);
    }

    for (int i = 0; i < scalingPower; ++i) {
        result = multiply(result, result);
    }
    return result;
}

void printMatrix(const std::string& name, const Matrix2& A) {
    std::cout << name << " =\n";
    for (const auto& row : A) {
        std::cout << "  [" << std::setw(12) << row[0] << ", "
                  << std::setw(12) << row[1] << "]\n";
    }
}

int main() {
    Matrix2 A = {{{0.0, 1.0}, {-4.0, -0.8}}};
    double t = 0.7;
    double s = 1.3;
    Vector2 x0 = {1.0, -0.25};

    Matrix2 Phi_t = matrixExpTaylor2(A, t);
    Matrix2 Phi_s = matrixExpTaylor2(A, s);
    Matrix2 Phi_t_plus_s = matrixExpTaylor2(A, t + s);
    Matrix2 Phi_minus_t = matrixExpTaylor2(A, -t);

    Matrix2 semigroupResidual = subtract(Phi_t_plus_s, multiply(Phi_t, Phi_s));
    Matrix2 inverseResidual = subtract(inverse2(Phi_t), Phi_minus_t);
    Matrix2 identityResidual = subtract(multiply(Phi_t, Phi_minus_t), identity2());

    Vector2 x_direct = multiply(Phi_t_plus_s, x0);
    Vector2 x_two_step = multiply(Phi_t, multiply(Phi_s, x0));

    std::cout << std::setprecision(10);
    printMatrix("Phi(t)", Phi_t);
    printMatrix("Phi(s)", Phi_s);
    printMatrix("Phi(t+s)", Phi_t_plus_s);
    std::cout << "Semigroup error = " << frobeniusNorm(semigroupResidual) << "\n";
    std::cout << "Inverse error = " << frobeniusNorm(inverseResidual) << "\n";
    std::cout << "Identity error = " << frobeniusNorm(identityResidual) << "\n";
    std::cout << "State two-step propagation error = " << vectorNorm(subtract(x_direct, x_two_step)) << "\n";
    return 0;
}
