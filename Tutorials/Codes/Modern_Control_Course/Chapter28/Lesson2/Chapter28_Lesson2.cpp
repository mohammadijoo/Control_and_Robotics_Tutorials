// Chapter28_Lesson2.cpp
// From-scratch computation of energy-like measures for a second-order state-space model.
// Typical C++ control/numerics libraries to know: Eigen, Armadillo, Blaze, SLICOT bindings.
// This file avoids external dependencies so it can be compiled with a standard C++17 compiler.

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

using Vec2 = std::array<double, 2>;

struct Sample {
    double t;
    Vec2 x;
    double u;
};

double input(double t) {
    return 0.5 * std::sin(3.0 * t) * std::exp(-0.2 * t);
}

Vec2 dynamics(double t, const Vec2& x) {
    double u = input(t);
    // x_dot = A x + B u, A = [[0, 1], [-4, -0.8]], B = [0, 1]^T
    return {x[1], -4.0 * x[0] - 0.8 * x[1] + u};
}

Vec2 add_scaled(const Vec2& x, const Vec2& k, double h) {
    return {x[0] + h * k[0], x[1] + h * k[1]};
}

Vec2 rk4_step(double t, const Vec2& x, double h) {
    Vec2 k1 = dynamics(t, x);
    Vec2 k2 = dynamics(t + 0.5 * h, add_scaled(x, k1, 0.5 * h));
    Vec2 k3 = dynamics(t + 0.5 * h, add_scaled(x, k2, 0.5 * h));
    Vec2 k4 = dynamics(t + h, add_scaled(x, k3, h));
    return {x[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
            x[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0};
}

double state_quadratic(const Vec2& x) {
    // x^T Q x with Q = diag(10, 1)
    return 10.0 * x[0] * x[0] + x[1] * x[1];
}

double output_weighted_quadratic(const Vec2& y) {
    // y^T W y with W = diag(1, 0.1)
    return y[0] * y[0] + 0.1 * y[1] * y[1];
}

double norm2(const Vec2& y) {
    return std::sqrt(y[0] * y[0] + y[1] * y[1]);
}

int main() {
    const double t0 = 0.0;
    const double tf = 12.0;
    const double h = 0.01;
    const int n = static_cast<int>((tf - t0) / h);

    std::vector<Sample> samples;
    samples.reserve(n + 1);

    Vec2 x = {1.0, 0.0};
    double t = t0;
    for (int i = 0; i <= n; ++i) {
        samples.push_back({t, x, input(t)});
        x = rk4_step(t, x, h);
        t += h;
    }

    double stateEnergy = 0.0;
    double inputEnergy = 0.0;
    double outputEnergy = 0.0;
    double linf = 0.0;

    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        const auto& a = samples[i];
        const auto& b = samples[i + 1];
        double dt = b.t - a.t;

        stateEnergy += 0.5 * dt * (state_quadratic(a.x) + state_quadratic(b.x));
        inputEnergy += 0.5 * dt * (0.2 * a.u * a.u + 0.2 * b.u * b.u);
        outputEnergy += 0.5 * dt * (output_weighted_quadratic(a.x) + output_weighted_quadratic(b.x));
        linf = std::max(linf, norm2(a.x));
    }

    double outputL2 = std::sqrt(outputEnergy);
    double outputRms = std::sqrt(outputEnergy / (tf - t0));

    std::cout << "Weighted state energy   = " << stateEnergy << "\n";
    std::cout << "Weighted input energy   = " << inputEnergy << "\n";
    std::cout << "Performance J           = " << stateEnergy + inputEnergy << "\n";
    std::cout << "Weighted output L2 norm = " << outputL2 << "\n";
    std::cout << "Weighted output RMS     = " << outputRms << "\n";
    std::cout << "Output L-infinity norm  = " << linf << "\n";
    return 0;
}
