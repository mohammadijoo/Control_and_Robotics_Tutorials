/*
Chapter21_Lesson5.cpp
Scratch C++ demonstration of inverse response caused by an RHP zero.

Compile, for example:
    g++ -std=c++17 -O2 Chapter21_Lesson5.cpp -o Chapter21_Lesson5

The transfer functions are represented by the controllable canonical realization
for denominator s^2 + 5s + 6:
    x1_dot = x2
    x2_dot = -6 x1 - 5 x2 + u
    y      = b0 x1 + b1 x2
where numerator is b1*s + b0.
*/

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

struct State {
    double x1;
    double x2;
};

State derivative(const State& x, double u) {
    return State{x.x2, -6.0 * x.x1 - 5.0 * x.x2 + u};
}

State add_scaled(const State& x, const State& k, double h) {
    return State{x.x1 + h * k.x1, x.x2 + h * k.x2};
}

State rk4_step(const State& x, double u, double dt) {
    State k1 = derivative(x, u);
    State k2 = derivative(add_scaled(x, k1, dt / 2.0), u);
    State k3 = derivative(add_scaled(x, k2, dt / 2.0), u);
    State k4 = derivative(add_scaled(x, k3, dt), u);
    return State{
        x.x1 + dt * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1) / 6.0,
        x.x2 + dt * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2) / 6.0
    };
}

double output(const State& x, double b0, double b1) {
    return b0 * x.x1 + b1 * x.x2;
}

double zero_location(double b0, double b1) {
    // Numerator b1*s + b0 has zero at -b0/b1.
    return -b0 / b1;
}

int main() {
    const double dt = 0.002;
    const double tf = 8.0;
    const double u = 1.0;

    // G_min(s)=6(s+1)/((s+2)(s+3)) => numerator 6s+6.
    const double b0_min = 6.0;
    const double b1_min = 6.0;

    // G_nmp(s)=6(1-s)/((s+2)(s+3)) => numerator -6s+6.
    const double b0_nmp = 6.0;
    const double b1_nmp = -6.0;

    std::cout << "Minimum-phase zero: " << zero_location(b0_min, b1_min) << "\n";
    std::cout << "Non-minimum-phase zero: " << zero_location(b0_nmp, b1_nmp) << "\n";
    std::cout << "Common poles: -2, -3\n";

    std::ofstream csv("Chapter21_Lesson5_cpp_step_response.csv");
    csv << "t,y_minimum_phase,y_nonminimum_phase\n";

    State xmin{0.0, 0.0};
    State xnmp{0.0, 0.0};
    double negative_area = 0.0;

    for (int i = 0; i <= static_cast<int>(tf / dt); ++i) {
        double t = i * dt;
        double ymin = output(xmin, b0_min, b1_min);
        double ynmp = output(xnmp, b0_nmp, b1_nmp);
        csv << std::fixed << std::setprecision(6) << t << "," << ymin << "," << ynmp << "\n";
        if (ynmp < 0.0) {
            negative_area += -ynmp * dt;
        }
        xmin = rk4_step(xmin, u, dt);
        xnmp = rk4_step(xnmp, u, dt);
    }

    std::cout << "Wrote Chapter21_Lesson5_cpp_step_response.csv\n";
    std::cout << "Approximate negative undershoot area: " << negative_area << "\n";
    return 0;
}
