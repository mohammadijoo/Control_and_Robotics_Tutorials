// Chapter18_Lesson3.cpp
// Port-Hamiltonian mass-spring-damper simulation using RK4
// Compile: g++ -O2 -std=c++17 Chapter18_Lesson3.cpp -o Chapter18_Lesson3

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

struct State {
    double q;
    double p;
};

const double m = 1.5;
const double k = 12.0;
const double d = 0.8;
const double omega_u = 1.4;

double input_u(double t) {
    return std::sin(omega_u * t);
}

double H(const State& x) {
    return 0.5 * k * x.q * x.q + 0.5 * x.p * x.p / m;
}

double y_output(const State& x) {
    return x.p / m;
}

double dissipation(const State& x) {
    // gradH^T R gradH = d * (p/m)^2
    double v = x.p / m;
    return d * v * v;
}

State dynamics(double t, const State& x) {
    State dx;
    // qdot = p/m
    dx.q = x.p / m;
    // pdot = -k q - d*(p/m) + u
    dx.p = -k * x.q - d * (x.p / m) + input_u(t);
    return dx;
}

State add(const State& a, const State& b_) {
    return {a.q + b_.q, a.p + b_.p};
}

State scale(double s, const State& x) {
    return {s * x.q, s * x.p};
}

State rk4_step(double t, const State& x, double h) {
    State k1 = dynamics(t, x);
    State k2 = dynamics(t + 0.5 * h, add(x, scale(0.5 * h, k1)));
    State k3 = dynamics(t + 0.5 * h, add(x, scale(0.5 * h, k2)));
    State k4 = dynamics(t + h, add(x, scale(h, k3)));
    State sum = add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4));
    return add(x, scale(h / 6.0, sum));
}

int main() {
    const double T = 20.0;
    const double h = 0.002;
    const int N = static_cast<int>(T / h) + 1;

    State x{0.15, 0.0};
    double H0 = H(x);
    double rhs_int = 0.0;

    std::ofstream csv("Chapter18_Lesson3_cpp_output.csv");
    csv << "t,q,p,H,u,y,dissipation,supply\n";
    csv << std::setprecision(12);

    double prev_rhs = 0.0;
    bool first = true;

    for (int i = 0; i < N; ++i) {
        double t = i * h;
        double u = input_u(t);
        double y = y_output(x);
        double diss = dissipation(x);
        double supply = y * u;
        double rhs = -diss + supply;

        if (first) {
            first = false;
        } else {
            rhs_int += 0.5 * h * (prev_rhs + rhs);
        }
        prev_rhs = rhs;

        csv << t << "," << x.q << "," << x.p << "," << H(x) << ","
            << u << "," << y << "," << diss << "," << supply << "\n";

        if (i < N - 1) {
            x = rk4_step(t, x, h);
        }
    }

    csv.close();

    double Hf = H(x);
    double residual = (Hf - H0) - rhs_int;

    std::cout << "Final state [q, p] = [" << x.q << ", " << x.p << "]\n";
    std::cout << "Initial energy     = " << H0 << "\n";
    std::cout << "Final energy       = " << Hf << "\n";
    std::cout << "Energy balance residual = " << residual << "\n";
    std::cout << "CSV written to Chapter18_Lesson3_cpp_output.csv\n";
    return 0;
}
