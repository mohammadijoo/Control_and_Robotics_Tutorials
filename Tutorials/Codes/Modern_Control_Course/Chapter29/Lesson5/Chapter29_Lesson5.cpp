/*
Chapter29_Lesson5.cpp

Slowly time-varying mass-spring-damper example.
No external C++ libraries are required.

Compile:
    g++ -std=c++17 -O2 Chapter29_Lesson5.cpp -o Chapter29_Lesson5

Run:
    ./Chapter29_Lesson5
*/

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

struct State {
    double x1;
    double x2;
};

const double EPSILON = 0.03;

double mass(double t) {
    return 1.0 + 0.12 * std::sin(EPSILON * t);
}

double damping(double t) {
    return 0.22 + 0.04 * std::cos(EPSILON * t);
}

double stiffness(double t) {
    return 2.0 + 0.30 * std::sin(0.5 * EPSILON * t);
}

double inputForce(double t) {
    return 0.2 * std::sin(0.7 * t);
}

State derivative(double t, State x) {
    double m = mass(t);
    double c = damping(t);
    double k = stiffness(t);
    double u = inputForce(t);

    State dx;
    dx.x1 = x.x2;
    dx.x2 = -(k / m) * x.x1 - (c / m) * x.x2 + (1.0 / m) * u;
    return dx;
}

State add(State a, State b, double scale) {
    return State{a.x1 + scale * b.x1, a.x2 + scale * b.x2};
}

State rk4Step(double t, State x, double dt) {
    State k1 = derivative(t, x);
    State k2 = derivative(t + 0.5 * dt, add(x, k1, 0.5 * dt));
    State k3 = derivative(t + 0.5 * dt, add(x, k2, 0.5 * dt));
    State k4 = derivative(t + dt, add(x, k3, dt));

    State next;
    next.x1 = x.x1 + (dt / 6.0) * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1);
    next.x2 = x.x2 + (dt / 6.0) * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2);
    return next;
}

double frozenDecayMargin(double t) {
    double m = mass(t);
    double c = damping(t);
    double k = stiffness(t);

    double trace = -c / m;
    double determinant = k / m;
    double discriminant = trace * trace - 4.0 * determinant;

    if (discriminant >= 0.0) {
        double lambda1 = 0.5 * (trace + std::sqrt(discriminant));
        double lambda2 = 0.5 * (trace - std::sqrt(discriminant));
        return -std::max(lambda1, lambda2);
    }

    return -0.5 * trace;
}

int main() {
    const double t0 = 0.0;
    const double tf = 160.0;
    const double dt = 0.01;
    const int steps = static_cast<int>((tf - t0) / dt);

    State x{1.0, 0.0};
    std::ofstream file("Chapter29_Lesson5_cpp_output.csv");
    file << "t,x1,x2,m,c,k,alpha\n";
    file << std::fixed << std::setprecision(10);

    for (int i = 0; i <= steps; ++i) {
        double t = t0 + i * dt;
        file << t << "," << x.x1 << "," << x.x2 << ","
             << mass(t) << "," << damping(t) << "," << stiffness(t) << ","
             << frozenDecayMargin(t) << "\n";

        if (i < steps) {
            x = rk4Step(t, x, dt);
        }
    }

    file.close();

    std::cout << "Final state: x1 = " << x.x1 << ", x2 = " << x.x2 << "\n";
    std::cout << "CSV written to Chapter29_Lesson5_cpp_output.csv\n";
    return 0;
}
