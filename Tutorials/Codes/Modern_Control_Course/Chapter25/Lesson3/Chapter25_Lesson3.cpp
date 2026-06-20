/*
Chapter25_Lesson3.cpp

Trade-offs in SISO state-feedback for a second-order plant.

Compile:
    g++ -std=c++17 -O2 Chapter25_Lesson3.cpp -o Chapter25_Lesson3

No external libraries are required.
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

struct Vec2 {
    double x1;
    double x2;
};

struct Mat2 {
    double a11;
    double a12;
    double a21;
    double a22;
};

Vec2 add(Vec2 a, Vec2 b) {
    return {a.x1 + b.x1, a.x2 + b.x2};
}

Vec2 scale(Vec2 a, double c) {
    return {c * a.x1, c * a.x2};
}

Vec2 mat_vec(Mat2 A, Vec2 x) {
    return {A.a11 * x.x1 + A.a12 * x.x2, A.a21 * x.x1 + A.a22 * x.x2};
}

double control(double k1, double k2, Vec2 x) {
    return -(k1 * x.x1 + k2 * x.x2);
}

Vec2 rk4_step(Mat2 Ac, Vec2 x, double dt) {
    Vec2 k1 = mat_vec(Ac, x);
    Vec2 k2 = mat_vec(Ac, add(x, scale(k1, 0.5 * dt)));
    Vec2 k3 = mat_vec(Ac, add(x, scale(k2, 0.5 * dt)));
    Vec2 k4 = mat_vec(Ac, add(x, scale(k3, dt)));

    Vec2 sum = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
    return add(x, scale(sum, dt / 6.0));
}

struct Metrics {
    double k1;
    double k2;
    double speed;
    double gain_norm;
    double effort_integral;
    double max_abs_u;
};

Metrics simulate_design(double desired_real_part, double desired_imag_part) {
    // Plant:
    // x_dot = [0 1; -2 -0.4] x + [0;1] u
    // u = -[k1 k2] x
    //
    // Desired poles: -a +- j b
    // desired characteristic polynomial:
    // s^2 + 2 a s + (a^2 + b^2)
    const double plant_const = 2.0;
    const double plant_s_coeff = 0.4;

    double a = desired_real_part;
    double b = desired_imag_part;
    double desired_s_coeff = 2.0 * a;
    double desired_const = a * a + b * b;

    double k1 = desired_const - plant_const;
    double k2 = desired_s_coeff - plant_s_coeff;

    Mat2 Ac{0.0, 1.0, -plant_const - k1, -plant_s_coeff - k2};

    Vec2 x{1.0, 0.0};
    const double dt = 0.0005;
    const double tf = 10.0;
    int steps = static_cast<int>(tf / dt);

    double effort = 0.0;
    double max_abs_u = 0.0;

    for (int i = 0; i < steps; ++i) {
        double u = control(k1, k2, x);
        effort += u * u * dt;
        max_abs_u = std::max(max_abs_u, std::abs(u));
        x = rk4_step(Ac, x, dt);
    }

    return {k1, k2, a, std::sqrt(k1 * k1 + k2 * k2), effort, max_abs_u};
}

int main() {
    struct CaseData {
        std::string name;
        double a;
        double b;
    };

    std::vector<CaseData> cases = {
        {"slow", 1.0, 1.0},
        {"medium", 3.0, 3.0},
        {"fast", 6.0, 6.0}
    };

    std::cout << "State-feedback speed/effort trade-off\n";
    std::cout << std::setw(10) << "case"
              << std::setw(12) << "speed"
              << std::setw(12) << "k1"
              << std::setw(12) << "k2"
              << std::setw(14) << "||K||"
              << std::setw(16) << "int u^2 dt"
              << std::setw(12) << "max |u|" << "\n";

    for (const auto& c : cases) {
        Metrics m = simulate_design(c.a, c.b);
        std::cout << std::setw(10) << c.name
                  << std::setw(12) << m.speed
                  << std::setw(12) << m.k1
                  << std::setw(12) << m.k2
                  << std::setw(14) << m.gain_norm
                  << std::setw(16) << m.effort_integral
                  << std::setw(12) << m.max_abs_u << "\n";
    }

    return 0;
}
