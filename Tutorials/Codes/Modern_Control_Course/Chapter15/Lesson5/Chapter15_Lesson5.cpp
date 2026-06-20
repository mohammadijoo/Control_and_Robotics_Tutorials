// Chapter15_Lesson5.cpp
// From-scratch finite-horizon Gramian integration for a 2x2 example.
// Compile:
//     g++ -std=c++17 Chapter15_Lesson5.cpp -O2 -o Chapter15_Lesson5
// Run:
//     ./Chapter15_Lesson5

#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>

using Mat2 = std::array<std::array<double, 2>, 2>;
using Vec2 = std::array<double, 2>;

Mat2 zeros() {
    return {{{0.0, 0.0}, {0.0, 0.0}}};
}

Mat2 add(const Mat2& A, const Mat2& B) {
    Mat2 C = zeros();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

Mat2 scale(const Mat2& A, double s) {
    Mat2 C = zeros();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            C[i][j] = s * A[i][j];
    return C;
}

Mat2 mul(const Mat2& A, const Mat2& B) {
    Mat2 C = zeros();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Mat2 trans(const Mat2& A) {
    return {{{A[0][0], A[1][0]}, {A[0][1], A[1][1]}}};
}

double frobenius_norm(const Mat2& A) {
    double s = 0.0;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            s += A[i][j] * A[i][j];
    return std::sqrt(s);
}

Mat2 mat_exp_2x2(const Mat2& A, double t) {
    // Scaling-and-squaring with truncated Taylor series, sufficient for this demonstration.
    Mat2 At = scale(A, t);
    int squarings = 6;
    double factor = std::pow(2.0, squarings);
    Mat2 X = scale(At, 1.0 / factor);

    Mat2 E = {{{1.0, 0.0}, {0.0, 1.0}}};
    Mat2 term = E;
    for (int k = 1; k <= 24; ++k) {
        term = scale(mul(term, X), 1.0 / k);
        E = add(E, term);
    }

    for (int i = 0; i < squarings; ++i) {
        E = mul(E, E);
    }
    return E;
}

Mat2 observability_integrand(const Mat2& A, const Vec2& C, double t) {
    Mat2 E = mat_exp_2x2(A, t);
    Mat2 Q = {{{C[0] * C[0], C[0] * C[1]},
               {C[1] * C[0], C[1] * C[1]}}};
    return mul(mul(trans(E), Q), E);
}

Mat2 controllability_integrand(const Mat2& A, const Vec2& B, double t) {
    Mat2 E = mat_exp_2x2(A, t);
    Mat2 R = {{{B[0] * B[0], B[0] * B[1]},
               {B[1] * B[0], B[1] * B[1]}}};
    return mul(mul(E, R), trans(E));
}

Mat2 integrate_observability(const Mat2& A, const Vec2& C, double T, int steps) {
    Mat2 W = zeros();
    double dt = T / steps;
    for (int k = 0; k <= steps; ++k) {
        double weight = (k == 0 || k == steps) ? 0.5 : 1.0;
        W = add(W, scale(observability_integrand(A, C, k * dt), weight));
    }
    return scale(W, dt);
}

Mat2 integrate_controllability(const Mat2& A, const Vec2& B, double T, int steps) {
    Mat2 W = zeros();
    double dt = T / steps;
    for (int k = 0; k <= steps; ++k) {
        double weight = (k == 0 || k == steps) ? 0.5 : 1.0;
        W = add(W, scale(controllability_integrand(A, B, k * dt), weight));
    }
    return scale(W, dt);
}

void print_matrix(const Mat2& A) {
    std::cout << std::fixed << std::setprecision(8);
    for (const auto& row : A) {
        std::cout << "[ " << row[0] << "  " << row[1] << " ]\n";
    }
}

int main() {
    Mat2 A = {{{-1.0, 2.0}, {-3.0, -4.0}}};
    Mat2 AT = trans(A);
    Vec2 C = {1.0, 0.5};
    Vec2 Bdual = {1.0, 0.5};

    double T = 3.0;
    int steps = 4000;

    Mat2 Wo = integrate_observability(A, C, T, steps);
    Mat2 WcDual = integrate_controllability(AT, Bdual, T, steps);

    std::cout << "Finite-horizon observability Gramian W_o(A,C):\n";
    print_matrix(Wo);

    std::cout << "\nFinite-horizon controllability Gramian W_c(A^T,C^T):\n";
    print_matrix(WcDual);

    Mat2 D = add(Wo, scale(WcDual, -1.0));
    std::cout << "\nFrobenius difference: " << frobenius_norm(D) << "\n";

    return 0;
}
