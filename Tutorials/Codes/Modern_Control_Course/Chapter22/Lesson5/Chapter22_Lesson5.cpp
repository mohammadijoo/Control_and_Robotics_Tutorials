/*
Chapter22_Lesson5.cpp

From-scratch simulation of a saturated state-feedback controller for
the double-integrator system:
    x1_dot = x2
    x2_dot = u
    u = sat(-K x)

Compile:
    g++ -std=c++17 -O2 Chapter22_Lesson5.cpp -o Chapter22_Lesson5

Run:
    ./Chapter22_Lesson5

The program writes Chapter22_Lesson5_cpp_output.csv.
*/

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>

using State = std::array<double, 2>;

double saturate(double value, double lower, double upper) {
    return std::min(std::max(value, lower), upper);
}

double rateLimit(double requested, double previous, double maxRate, double dt) {
    const double step = maxRate * dt;
    return saturate(requested, previous - step, previous + step);
}

State dynamics(const State& x, double u) {
    return State{x[1], u};
}

State addScaled(const State& x, const State& dx, double scale) {
    return State{x[0] + scale * dx[0], x[1] + scale * dx[1]};
}

int main() {
    const double k1 = 6.0;
    const double k2 = 5.0;
    const double umax = 1.0;
    const double maxRate = 8.0;
    const double dt = 0.001;
    const double tf = 6.0;
    const int nSteps = static_cast<int>(tf / dt);

    State x{1.2, 0.0};
    double uPrev = 0.0;

    std::ofstream file("Chapter22_Lesson5_cpp_output.csv");
    file << "time,x1,x2,u_command,u_actual\n";

    for (int i = 0; i <= nSteps; ++i) {
        const double time = i * dt;
        const double uCommand = -(k1 * x[0] + k2 * x[1]);
        const double uSat = saturate(uCommand, -umax, umax);
        const double u = rateLimit(uSat, uPrev, maxRate, dt);
        uPrev = u;

        file << time << "," << x[0] << "," << x[1] << ","
             << uCommand << "," << u << "\n";

        // Fourth-order Runge-Kutta with held input u.
        State q1 = dynamics(x, u);
        State q2 = dynamics(addScaled(x, q1, 0.5 * dt), u);
        State q3 = dynamics(addScaled(x, q2, 0.5 * dt), u);
        State q4 = dynamics(addScaled(x, q3, dt), u);

        x[0] += (dt / 6.0) * (q1[0] + 2.0 * q2[0] + 2.0 * q3[0] + q4[0]);
        x[1] += (dt / 6.0) * (q1[1] + 2.0 * q2[1] + 2.0 * q3[1] + q4[1]);
    }

    std::cout << "Finished. CSV written to Chapter22_Lesson5_cpp_output.csv\n";
    return 0;
}
