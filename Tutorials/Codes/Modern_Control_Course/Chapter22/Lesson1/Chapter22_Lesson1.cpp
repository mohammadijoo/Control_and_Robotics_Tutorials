// Chapter22_Lesson1.cpp
// Structure of state-feedback law: u = -K x + r
// Compile: g++ -std=c++17 Chapter22_Lesson1.cpp -O2 -o Chapter22_Lesson1

#include <array>
#include <cmath>
#include <iostream>

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

Vec2 add(Vec2 a, Vec2 b) {
    return {a[0] + b[0], a[1] + b[1]};
}

Vec2 scale(double s, Vec2 a) {
    return {s * a[0], s * a[1]};
}

Vec2 mat_vec(Mat2 M, Vec2 x) {
    return {
        M[0][0] * x[0] + M[0][1] * x[1],
        M[1][0] * x[0] + M[1][1] * x[1]
    };
}

Vec2 dynamics(const Mat2& A, const Vec2& B, const Vec2& K, double r, Vec2 x) {
    double u = -K[0] * x[0] - K[1] * x[1] + r;
    Vec2 Ax = mat_vec(A, x);
    return {Ax[0] + B[0] * u, Ax[1] + B[1] * u};
}

Vec2 rk4_step(const Mat2& A, const Vec2& B, const Vec2& K, double r, double h, Vec2 x) {
    Vec2 k1 = dynamics(A, B, K, r, x);
    Vec2 k2 = dynamics(A, B, K, r, add(x, scale(0.5 * h, k1)));
    Vec2 k3 = dynamics(A, B, K, r, add(x, scale(0.5 * h, k2)));
    Vec2 k4 = dynamics(A, B, K, r, add(x, scale(h, k3)));

    return add(x, scale(h / 6.0, add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4))));
}

int main() {
    // Mass-spring-damper model: x1 = position, x2 = velocity
    Mat2 A = {{{0.0, 1.0}, {-2.0, -0.4}}};
    Vec2 B = {0.0, 1.0};
    Vec2 C = {1.0, 0.0};
    Vec2 K = {4.0, 2.6};
    double r = 1.0;

    // Closed-loop matrix Acl = A - B*K
    Mat2 Acl = A;
    for (int i = 0; i < 2; ++i) {
        Acl[i][0] -= B[i] * K[0];
        Acl[i][1] -= B[i] * K[1];
    }

    std::cout << "A - B K =\n";
    std::cout << Acl[0][0] << " " << Acl[0][1] << "\n";
    std::cout << Acl[1][0] << " " << Acl[1][1] << "\n";

    // For a 2x2 matrix, eigenvalues solve lambda^2 - tr lambda + det = 0.
    double tr = Acl[0][0] + Acl[1][1];
    double det = Acl[0][0] * Acl[1][1] - Acl[0][1] * Acl[1][0];
    double disc = tr * tr - 4.0 * det;
    std::cout << "trace = " << tr << ", determinant = " << det << "\n";
    if (disc >= 0.0) {
        std::cout << "eigenvalues = "
                  << 0.5 * (tr + std::sqrt(disc)) << ", "
                  << 0.5 * (tr - std::sqrt(disc)) << "\n";
    } else {
        std::cout << "eigenvalues = " << 0.5 * tr << " +/- "
                  << 0.5 * std::sqrt(-disc) << " i\n";
    }

    Vec2 x = {0.2, 0.0};
    double h = 0.01;
    double tf = 8.0;

    std::cout << "t,x1,x2,u,y\n";
    for (int step = 0; step <= static_cast<int>(tf / h); ++step) {
        double t = step * h;
        double u = -K[0] * x[0] - K[1] * x[1] + r;
        double y = C[0] * x[0] + C[1] * x[1];

        if (step % 100 == 0) {
            std::cout << t << "," << x[0] << "," << x[1] << "," << u << "," << y << "\n";
        }
        x = rk4_step(A, B, K, r, h, x);
    }

    return 0;
}
