/*
Chapter28_Lesson3.cpp
Modern Control — Chapter 28, Lesson 3
State and control weighting matrices Q and R as performance descriptors.

This standalone C++ example computes a finite-horizon weighted cost
for xdot = (A - B K)x, u = -Kx using RK4 integration.
*/

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

Vec2 mat_vec(const Mat2& A, const Vec2& x) {
    return {A[0][0] * x[0] + A[0][1] * x[1],
            A[1][0] * x[0] + A[1][1] * x[1]};
}

Vec2 add(const Vec2& a, const Vec2& b) {
    return {a[0] + b[0], a[1] + b[1]};
}

Vec2 scale(double c, const Vec2& x) {
    return {c * x[0], c * x[1]};
}

double quad2(const Vec2& x, const Mat2& Q) {
    return x[0] * (Q[0][0] * x[0] + Q[0][1] * x[1]) +
           x[1] * (Q[1][0] * x[0] + Q[1][1] * x[1]);
}

double input_value(const std::array<double, 2>& K, const Vec2& x) {
    return -(K[0] * x[0] + K[1] * x[1]);
}

Vec2 rhs(const Mat2& Acl, const Vec2& x) {
    return mat_vec(Acl, x);
}

Vec2 rk4_step(const Mat2& Acl, const Vec2& x, double h) {
    Vec2 k1 = rhs(Acl, x);
    Vec2 k2 = rhs(Acl, add(x, scale(0.5 * h, k1)));
    Vec2 k3 = rhs(Acl, add(x, scale(0.5 * h, k2)));
    Vec2 k4 = rhs(Acl, add(x, scale(h, k3)));
    return add(x, scale(h / 6.0, add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4))));
}

bool is_pd_1x1(double r) {
    return r > 0.0;
}

bool is_psd_2x2(const Mat2& Q) {
    // Sylvester condition for 2x2 symmetric PSD:
    // q11 >= 0, q22 >= 0, det(Q) >= 0.
    const double det = Q[0][0] * Q[1][1] - Q[0][1] * Q[1][0];
    return Q[0][0] >= -1e-12 && Q[1][1] >= -1e-12 && det >= -1e-12;
}

int main() {
    // Plant and feedback: xdot = (A - B K)x.
    Mat2 A = {{{0.0, 1.0}, {-2.0, -0.4}}};
    std::array<double, 2> B = {0.0, 1.0};
    std::array<double, 2> K = {3.0, 2.2};

    Mat2 Acl = A;
    Acl[0][0] -= B[0] * K[0];
    Acl[0][1] -= B[0] * K[1];
    Acl[1][0] -= B[1] * K[0];
    Acl[1][1] -= B[1] * K[1];

    // Bryson-style tolerances: x1_max=1, x2_max=2, u_max=0.5.
    Mat2 Q = {{{1.0, 0.0}, {0.0, 1.0 / 4.0}}};
    double R = 1.0 / (0.5 * 0.5);

    if (!is_psd_2x2(Q)) {
        throw std::runtime_error("Q must be positive semidefinite.");
    }
    if (!is_pd_1x1(R)) {
        throw std::runtime_error("R must be positive definite.");
    }

    Vec2 x = {1.0, 0.0};
    double h = 0.001;
    double tf = 8.0;
    int steps = static_cast<int>(tf / h);
    double J = 0.0;

    auto integrand = [&](const Vec2& xv) {
        double u = input_value(K, xv);
        return quad2(xv, Q) + R * u * u;
    };

    double f_old = integrand(x);
    for (int k = 0; k < steps; ++k) {
        Vec2 x_next = rk4_step(Acl, x, h);
        double f_new = integrand(x_next);
        J += 0.5 * h * (f_old + f_new);
        x = x_next;
        f_old = f_new;
    }

    std::cout << "Q = [[1, 0], [0, 0.25]]\n";
    std::cout << "R = " << R << "\n";
    std::cout << "Finite-horizon weighted cost J_T = " << J << "\n";
    return 0;
}
