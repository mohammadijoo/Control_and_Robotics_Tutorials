// Chapter30_Lesson4.cpp
// Integrated case study: mass-spring-damper model to state-feedback controller
// This implementation uses only the C++ standard library. For larger systems, use Eigen or Armadillo.

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

Vec2 add(Vec2 a, Vec2 b) { return {a[0] + b[0], a[1] + b[1]}; }
Vec2 scale(double c, Vec2 a) { return {c * a[0], c * a[1]}; }
Vec2 matvec(const Mat2& A, Vec2 x) {
    return {A[0][0] * x[0] + A[0][1] * x[1], A[1][0] * x[0] + A[1][1] * x[1]};
}

double dot(Vec2 a, Vec2 b) { return a[0] * b[0] + a[1] * b[1]; }

Vec2 dynamics(const Mat2& Acl, Vec2 Bcl, double reference, Vec2 x) {
    Vec2 ax = matvec(Acl, x);
    return add(ax, scale(reference, Bcl));
}

Vec2 rk4Step(const Mat2& Acl, Vec2 Bcl, double reference, Vec2 x, double h) {
    Vec2 k1 = dynamics(Acl, Bcl, reference, x);
    Vec2 k2 = dynamics(Acl, Bcl, reference, add(x, scale(0.5 * h, k1)));
    Vec2 k3 = dynamics(Acl, Bcl, reference, add(x, scale(0.5 * h, k2)));
    Vec2 k4 = dynamics(Acl, Bcl, reference, add(x, scale(h, k3)));
    return add(x, scale(h / 6.0, add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4))));
}

int main() {
    const double m = 1.2;
    const double b = 0.8;
    const double k = 3.0;
    const double zeta = 0.70;
    const double settlingTime = 2.0;
    const double omegaN = 4.0 / (zeta * settlingTime);

    const Mat2 A = {{{0.0, 1.0}, {-k / m, -b / m}}};
    const Vec2 B = {0.0, 1.0 / m};
    const Vec2 C = {1.0, 0.0};

    const double k1 = m * omegaN * omegaN - k;
    const double k2 = 2.0 * zeta * omegaN * m - b;
    const Vec2 K = {k1, k2};
    const double nbar = k + k1;

    const Mat2 Acl = {{{0.0, 1.0}, {-(k + k1) / m, -(b + k2) / m}}};
    const Vec2 Bcl = {0.0, nbar / m};

    const double trace = Acl[0][0] + Acl[1][1];
    const double determinant = Acl[0][0] * Acl[1][1] - Acl[0][1] * Acl[1][0];

    std::cout << "K = [" << K[0] << ", " << K[1] << "]\n";
    std::cout << "Nbar = " << nbar << "\n";
    std::cout << "Closed-loop characteristic: s^2 - (" << trace << ") s + " << determinant << "\n";

    const double reference = 1.0;
    const double dt = 0.002;
    const double tFinal = 6.0;
    const int steps = static_cast<int>(tFinal / dt);
    Vec2 x = {0.0, 0.0};

    std::ofstream csv("Chapter30_Lesson4_response_cpp.csv");
    csv << "t,position,velocity,output,control\n";

    double maxAbsU = 0.0;
    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        double y = dot(C, x);
        double u = -dot(K, x) + nbar * reference;
        if (std::abs(u) > maxAbsU) maxAbsU = std::abs(u);
        csv << t << "," << x[0] << "," << x[1] << "," << y << "," << u << "\n";
        if (i < steps) x = rk4Step(Acl, Bcl, reference, x, dt);
    }

    std::cout << "Final output = " << dot(C, x) << "\n";
    std::cout << "Maximum absolute control input = " << maxAbsU << "\n";
    return 0;
}
