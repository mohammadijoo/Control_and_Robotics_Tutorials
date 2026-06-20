// Chapter29_Lesson1.cpp
// Linear time-varying (LTV) state-space simulation using fixed-step RK4.
// Compile:
//     g++ -std=c++17 -O2 Chapter29_Lesson1.cpp -o Chapter29_Lesson1

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

struct Matrices {
    Mat2 A;
    Vec2 B;
    std::array<double, 2> C;
    double D;
};

Matrices matrices(double t) {
    const double omega = 1.0 + 0.30 * std::sin(0.70 * t);
    const double damping = 0.15 + 0.05 * std::cos(0.50 * t);

    Matrices m{};
    m.A = {{
        {{0.0, 1.0}},
        {{-(omega * omega), -2.0 * damping * omega}}
    }};
    m.B = {{0.0, 1.0 + 0.20 * std::sin(0.30 * t)}};
    m.C = {{1.0 + 0.10 * std::sin(0.40 * t), 0.0}};
    m.D = 0.0;
    return m;
}

double inputSignal(double t) {
    return std::sin(1.2 * t);
}

Vec2 rhs(double t, const Vec2& x) {
    Matrices m = matrices(t);
    const double u = inputSignal(t);

    return {{
        m.A[0][0] * x[0] + m.A[0][1] * x[1] + m.B[0] * u,
        m.A[1][0] * x[0] + m.A[1][1] * x[1] + m.B[1] * u
    }};
}

Vec2 addScaled(const Vec2& x, const Vec2& k, double scale) {
    return {{x[0] + scale * k[0], x[1] + scale * k[1]}};
}

Vec2 rk4Step(double t, const Vec2& x, double h) {
    Vec2 k1 = rhs(t, x);
    Vec2 k2 = rhs(t + 0.5 * h, addScaled(x, k1, 0.5 * h));
    Vec2 k3 = rhs(t + 0.5 * h, addScaled(x, k2, 0.5 * h));
    Vec2 k4 = rhs(t + h, addScaled(x, k3, h));

    return {{
        x[0] + (h / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
        x[1] + (h / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    }};
}

int main() {
    const double t0 = 0.0;
    const double tf = 20.0;
    const double h = 0.01;

    Vec2 x = {{1.0, 0.0}};

    std::cout << std::fixed << std::setprecision(8);
    std::cout << "t,x1,x2,y\n";

    for (double t = t0; t <= tf + 1e-12; t += h) {
        Matrices m = matrices(t);
        double u = inputSignal(t);
        double y = m.C[0] * x[0] + m.C[1] * x[1] + m.D * u;

        if (static_cast<int>(std::round(t / h)) % 100 == 0) {
            std::cout << t << "," << x[0] << "," << x[1] << "," << y << "\n";
        }

        if (t + h <= tf + 1e-12) {
            x = rk4Step(t, x, h);
        }
    }

    return 0;
}
