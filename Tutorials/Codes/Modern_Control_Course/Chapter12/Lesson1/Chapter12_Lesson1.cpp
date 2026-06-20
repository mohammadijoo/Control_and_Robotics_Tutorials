// Chapter12_Lesson1.cpp
// Finite-horizon controllability Gramian for a 2 by 2 example.
// Compile:
//   g++ -std=c++17 Chapter12_Lesson1.cpp -O2 -o Chapter12_Lesson1
//
// This from-scratch implementation avoids external dependencies and uses RK4
// to integrate the matrix differential equation:
//   dW/dt = A W + W A^T + B B^T,   W(0) = 0.
// For production work, consider Eigen, Armadillo, or the SLICOT routines.

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Mat2 = std::array<std::array<double, 2>, 2>;
using Vec2 = std::array<double, 2>;

Mat2 zero2() {
    return {{{0.0, 0.0}, {0.0, 0.0}}};
}

Mat2 add(const Mat2& X, const Mat2& Y) {
    Mat2 Z = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            Z[i][j] = X[i][j] + Y[i][j];
    return Z;
}

Mat2 scale(const Mat2& X, double a) {
    Mat2 Z = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            Z[i][j] = a * X[i][j];
    return Z;
}

Mat2 mul(const Mat2& X, const Mat2& Y) {
    Mat2 Z = zero2();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                Z[i][j] += X[i][k] * Y[k][j];
    return Z;
}

Mat2 transpose(const Mat2& X) {
    return {{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}}};
}

Mat2 outer(const Vec2& b) {
    return {{{b[0] * b[0], b[0] * b[1]}, {b[1] * b[0], b[1] * b[1]}}};
}

Mat2 gramian_rhs(const Mat2& A, const Vec2& b, const Mat2& W) {
    Mat2 AT = transpose(A);
    Mat2 BBt = outer(b);
    return add(add(mul(A, W), mul(W, AT)), BBt);
}

Mat2 rk4_step(const Mat2& A, const Vec2& b, const Mat2& W, double h) {
    Mat2 k1 = gramian_rhs(A, b, W);
    Mat2 k2 = gramian_rhs(A, b, add(W, scale(k1, 0.5 * h)));
    Mat2 k3 = gramian_rhs(A, b, add(W, scale(k2, 0.5 * h)));
    Mat2 k4 = gramian_rhs(A, b, add(W, scale(k3, h)));

    Mat2 update = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
    return add(W, scale(update, h / 6.0));
}

Mat2 finite_horizon_gramian(const Mat2& A, const Vec2& b, double T, int steps) {
    Mat2 W = zero2();
    double h = T / static_cast<double>(steps);
    for (int k = 0; k < steps; ++k) {
        W = rk4_step(A, b, W, h);
    }

    // Symmetrize to reduce small numerical skew.
    W[0][1] = W[1][0] = 0.5 * (W[0][1] + W[1][0]);
    return W;
}

double det2(const Mat2& X) {
    return X[0][0] * X[1][1] - X[0][1] * X[1][0];
}

void print_matrix(const char* name, const Mat2& X) {
    std::cout << "\n" << name << " =\n";
    for (int i = 0; i < 2; ++i) {
        std::cout << "[ ";
        for (int j = 0; j < 2; ++j) {
            std::cout << std::setw(12) << std::setprecision(7) << std::fixed << X[i][j] << " ";
        }
        std::cout << "]\n";
    }
}

int main() {
    Mat2 A = {{{0.0, 1.0}, {-2.0, -3.0}}};
    Vec2 b = {0.0, 1.0};

    double T = 2.0;
    int steps = 20000;

    Mat2 W = finite_horizon_gramian(A, b, T, steps);
    print_matrix("Wc(T)", W);

    std::cout << "\ndet(Wc(T)) = " << std::setprecision(12) << det2(W) << "\n";
    if (det2(W) > 1e-10) {
        std::cout << "The 2 by 2 Gramian is nonsingular: the system is controllable on this horizon.\n";
    } else {
        std::cout << "The Gramian is singular or nearly singular: controllability is lost or ill-conditioned.\n";
    }

    return 0;
}
