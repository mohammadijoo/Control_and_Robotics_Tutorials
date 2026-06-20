// Chapter27_Lesson1.cpp
// Formulating Tracking Problems in State-Space Form
//
// Compile:
//   g++ -std=c++17 Chapter27_Lesson1.cpp -o Chapter27_Lesson1
//
// This from-scratch implementation solves the constant tracking equilibrium
// equations for a second-order plant and simulates a deviation-feedback
// tracking formulation using RK4 integration.

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;

struct Equilibrium {
    double x1_bar;
    double x2_bar;
    double u_bar;
    double residual_norm;
};

Vec3 solve3x3(double M[3][3], Vec3 b) {
    // Gaussian elimination with partial pivoting.
    for (int k = 0; k < 3; ++k) {
        int pivot = k;
        double best = std::abs(M[k][k]);
        for (int i = k + 1; i < 3; ++i) {
            if (std::abs(M[i][k]) > best) {
                best = std::abs(M[i][k]);
                pivot = i;
            }
        }
        if (pivot != k) {
            for (int j = 0; j < 3; ++j) std::swap(M[k][j], M[pivot][j]);
            std::swap(b[k], b[pivot]);
        }

        const double diag = M[k][k];
        if (std::abs(diag) < 1e-12) {
            throw std::runtime_error("Singular equilibrium matrix.");
        }

        for (int j = k; j < 3; ++j) M[k][j] /= diag;
        b[k] /= diag;

        for (int i = 0; i < 3; ++i) {
            if (i == k) continue;
            const double factor = M[i][k];
            for (int j = k; j < 3; ++j) M[i][j] -= factor * M[k][j];
            b[i] -= factor * b[k];
        }
    }
    return b;
}

Equilibrium compute_equilibrium(double reference, double disturbance) {
    // Plant:
    //   x1_dot = x2
    //   x2_dot = -2 x1 - 0.8 x2 + u + d
    //   y      = x1
    //
    // Constant tracking equilibrium:
    //   0 = x2
    //   0 = -2 x1 - 0.8 x2 + u + d
    //   r = x1

    double M[3][3] = {
        {0.0, 1.0, 0.0},
        {-2.0, -0.8, 1.0},
        {1.0, 0.0, 0.0}
    };
    Vec3 rhs = {0.0, -disturbance, reference};

    Vec3 theta = solve3x3(M, rhs);
    const double x1 = theta[0];
    const double x2 = theta[1];
    const double u = theta[2];

    const double res1 = x2;
    const double res2 = -2.0 * x1 - 0.8 * x2 + u + disturbance;
    const double res3 = x1 - reference;
    const double norm = std::sqrt(res1 * res1 + res2 * res2 + res3 * res3);

    return {x1, x2, u, norm};
}

Vec2 dynamics(const Vec2& x, const Equilibrium& eq, double disturbance) {
    const double k1 = 4.0;
    const double k2 = 2.2;
    const double u = eq.u_bar - k1 * (x[0] - eq.x1_bar) - k2 * (x[1] - eq.x2_bar);

    Vec2 dx;
    dx[0] = x[1];
    dx[1] = -2.0 * x[0] - 0.8 * x[1] + u + disturbance;
    return dx;
}

Vec2 add_scaled(const Vec2& x, const Vec2& k, double scale) {
    return {x[0] + scale * k[0], x[1] + scale * k[1]};
}

Vec2 rk4_step(const Vec2& x, double h, const Equilibrium& eq, double disturbance) {
    Vec2 k1 = dynamics(x, eq, disturbance);
    Vec2 k2 = dynamics(add_scaled(x, k1, 0.5 * h), eq, disturbance);
    Vec2 k3 = dynamics(add_scaled(x, k2, 0.5 * h), eq, disturbance);
    Vec2 k4 = dynamics(add_scaled(x, k3, h), eq, disturbance);

    return {
        x[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
        x[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
    };
}

int main() {
    const double reference = 1.0;
    const double disturbance = 0.2;
    Equilibrium eq = compute_equilibrium(reference, disturbance);

    std::cout << std::fixed << std::setprecision(8);
    std::cout << "x_bar = [" << eq.x1_bar << ", " << eq.x2_bar << "]\n";
    std::cout << "u_bar = " << eq.u_bar << "\n";
    std::cout << "equilibrium residual norm = " << eq.residual_norm << "\n";

    Vec2 x = {0.0, 0.0};
    const double h = 0.001;
    const double tf = 8.0;
    const int steps = static_cast<int>(tf / h);

    for (int i = 0; i < steps; ++i) {
        x = rk4_step(x, h, eq, disturbance);
    }

    const double y = x[0];
    const double e = y - reference;

    std::cout << "final y(T) = " << y << "\n";
    std::cout << "final e(T) = " << e << "\n";
    return 0;
}
