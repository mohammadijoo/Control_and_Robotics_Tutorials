/*
Chapter30_Lesson1.cpp
Index-1 descriptor system reduction for E xdot = A x + B u.

This standalone C++ example avoids external libraries. It uses the reduced
ODE obtained from the algebraic constraint and simulates it with RK4.
*/

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

constexpr Mat2 Ar = {{{0.0, 1.0}, {-3.0, -3.0}}};
constexpr Vec2 Br = {0.0, 2.0};

double u_of_t(double t) {
    return (t >= 1.0) ? 1.0 : 0.0;
}

Vec2 f(double t, const Vec2& x) {
    const double u = u_of_t(t);
    return {
        Ar[0][0] * x[0] + Ar[0][1] * x[1] + Br[0] * u,
        Ar[1][0] * x[0] + Ar[1][1] * x[1] + Br[1] * u
    };
}

Vec2 add_scaled(const Vec2& x, const Vec2& k, double scale) {
    return {x[0] + scale * k[0], x[1] + scale * k[1]};
}

Vec2 rk4_step(double t, const Vec2& x, double h) {
    Vec2 k1 = f(t, x);
    Vec2 k2 = f(t + 0.5 * h, add_scaled(x, k1, 0.5 * h));
    Vec2 k3 = f(t + 0.5 * h, add_scaled(x, k2, 0.5 * h));
    Vec2 k4 = f(t + h, add_scaled(x, k3, h));
    return {
        x[0] + (h / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
        x[1] + (h / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    };
}

double algebraic_x3(const Vec2& xd, double u) {
    // Constraint: 0 = x1 + x3 - u, hence x3 = -x1 + u.
    return -xd[0] + u;
}

void print_reduced_eigenvalues() {
    // Eigenvalues of a 2 by 2 matrix using the quadratic formula.
    const double trace = Ar[0][0] + Ar[1][1];
    const double determinant = Ar[0][0] * Ar[1][1] - Ar[0][1] * Ar[1][0];
    const double discriminant = trace * trace - 4.0 * determinant;
    std::cout << "Reduced characteristic polynomial: lambda^2 - (trace) lambda + det" << std::endl;
    if (discriminant >= 0.0) {
        const double r1 = 0.5 * (trace + std::sqrt(discriminant));
        const double r2 = 0.5 * (trace - std::sqrt(discriminant));
        std::cout << "Reduced eigenvalues: " << r1 << ", " << r2 << std::endl;
    } else {
        const double real = 0.5 * trace;
        const double imag = 0.5 * std::sqrt(-discriminant);
        std::cout << "Reduced eigenvalues: " << real << " + " << imag << "i, "
                  << real << " - " << imag << "i" << std::endl;
    }
}

int main() {
    print_reduced_eigenvalues();

    Vec2 xd = {0.2, 0.0};
    const double h = 0.01;
    const int steps = 800;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "time,x1,x2,x3,constraint_residual" << std::endl;

    for (int k = 0; k <= steps; ++k) {
        const double t = k * h;
        const double u = u_of_t(t);
        const double x3 = algebraic_x3(xd, u);
        const double residual = xd[0] + x3 - u;

        if (k % 40 == 0) {
            std::cout << t << "," << xd[0] << "," << xd[1] << "," << x3 << "," << residual << std::endl;
        }
        xd = rk4_step(t, xd, h);
    }
    return 0;
}
