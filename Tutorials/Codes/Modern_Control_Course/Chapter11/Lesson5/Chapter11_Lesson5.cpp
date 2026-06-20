/*
Chapter11_Lesson5.cpp
Small fixed-size C++ implementation for an LTV controllability Gramian.
Compile for example:
    g++ -std=c++17 -O2 Chapter11_Lesson5.cpp -o Chapter11_Lesson5
*/

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Mat2 = std::array<std::array<double, 2>, 2>;
using Vec8 = std::array<double, 8>;

Mat2 A(double t) {
    return {{{0.0, 1.0}, {-(2.0 + 0.60 * std::sin(t)), -0.25}}};
}

std::array<double, 2> B(double t) {
    return {{0.0, 1.0 + 0.45 * std::cos(t)}};
}

Mat2 add(const Mat2& X, const Mat2& Y) {
    Mat2 Z{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            Z[i][j] = X[i][j] + Y[i][j];
    return Z;
}

Mat2 mul(const Mat2& X, const Mat2& Y) {
    Mat2 Z{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                Z[i][j] += X[i][k] * Y[k][j];
    return Z;
}

Mat2 transpose(const Mat2& X) {
    return {{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}}};
}

Mat2 outerBB(double t) {
    auto b = B(t);
    return {{{b[0] * b[0], b[0] * b[1]}, {b[1] * b[0], b[1] * b[1]}}};
}

Vec8 derivative(double t, const Vec8& z) {
    Mat2 Phi{{{z[0], z[1]}, {z[2], z[3]}}};
    Mat2 W{{{z[4], z[5]}, {z[6], z[7]}}};
    Mat2 At = A(t);
    Mat2 dPhi = mul(At, Phi);
    Mat2 dW = add(add(mul(At, W), mul(W, transpose(At))), outerBB(t));
    return {{dPhi[0][0], dPhi[0][1], dPhi[1][0], dPhi[1][1],
             dW[0][0], dW[0][1], dW[1][0], dW[1][1]}};
}

Vec8 rk4_step(double t, const Vec8& z, double h) {
    Vec8 k1 = derivative(t, z), z2{}, z3{}, z4{};
    for (int i = 0; i < 8; ++i) z2[i] = z[i] + 0.5 * h * k1[i];
    Vec8 k2 = derivative(t + 0.5 * h, z2);
    for (int i = 0; i < 8; ++i) z3[i] = z[i] + 0.5 * h * k2[i];
    Vec8 k3 = derivative(t + 0.5 * h, z3);
    for (int i = 0; i < 8; ++i) z4[i] = z[i] + h * k3[i];
    Vec8 k4 = derivative(t + h, z4);

    Vec8 zn{};
    for (int i = 0; i < 8; ++i)
        zn[i] = z[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
    return zn;
}

int main() {
    const double t0 = 0.0, tf = 4.0;
    const int steps = 4000;
    const double h = (tf - t0) / steps;

    Vec8 z = {{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}};
    double t = t0;
    for (int k = 0; k < steps; ++k) {
        z = rk4_step(t, z, h);
        t += h;
    }

    Mat2 W{{{z[4], 0.5 * (z[5] + z[6])}, {0.5 * (z[5] + z[6]), z[7]}}};
    double trace = W[0][0] + W[1][1];
    double det = W[0][0] * W[1][1] - W[0][1] * W[1][0];
    double disc = std::sqrt(std::max(0.0, trace * trace - 4.0 * det));
    double lambda_min = 0.5 * (trace - disc);

    std::cout << std::setprecision(10);
    std::cout << "Wc(0,4) =\n";
    std::cout << W[0][0] << "  " << W[0][1] << "\n";
    std::cout << W[1][0] << "  " << W[1][1] << "\n";
    std::cout << "det(Wc) = " << det << "\n";
    std::cout << "lambda_min(Wc) = " << lambda_min << "\n";
    std::cout << "Controllable? " << (lambda_min > 1e-8 ? "yes" : "no") << "\n";
    return 0;
}
