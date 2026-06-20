/*
Chapter27_Lesson5.cpp

Self-contained C++ case study for reference tracking and disturbance rejection.
No external control library is required.

Plant:
    q_dot = v
    v_dot = (-k q - b v + u + d)/m
    y = q

Feedforward:
    u = -K1 q - K2 v + Nbar r

Integral servo:
    z_dot = r - q
    u = -K1 q - K2 v + Ki z

Compile:
    g++ -std=c++17 Chapter27_Lesson5.cpp -O2 -o Chapter27_Lesson5
*/

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

struct Params {
    double m = 1.0;
    double b = 0.6;
    double k = 2.0;
    double r = 1.0;
    double d = 0.4;
};

struct State2 {
    double q;
    double v;
};

struct State3 {
    double q;
    double v;
    double z;
};

State2 deriv_feedforward(const State2& x, const Params& p, double K1, double K2, double nbar, double d) {
    double u = -K1 * x.q - K2 * x.v + nbar * p.r;
    return {x.v, (-p.k * x.q - p.b * x.v + u + d) / p.m};
}

State3 deriv_integral(const State3& x, const Params& p, double K1, double K2, double Ki, double d) {
    double u = -K1 * x.q - K2 * x.v + Ki * x.z;
    return {x.v, (-p.k * x.q - p.b * x.v + u + d) / p.m, p.r - x.q};
}

State2 rk4_feedforward_step(const State2& x, const Params& p, double h, double K1, double K2, double nbar, double d) {
    State2 k1 = deriv_feedforward(x, p, K1, K2, nbar, d);
    State2 x2{x.q + 0.5 * h * k1.q, x.v + 0.5 * h * k1.v};
    State2 k2 = deriv_feedforward(x2, p, K1, K2, nbar, d);
    State2 x3{x.q + 0.5 * h * k2.q, x.v + 0.5 * h * k2.v};
    State2 k3 = deriv_feedforward(x3, p, K1, K2, nbar, d);
    State2 x4{x.q + h * k3.q, x.v + h * k3.v};
    State2 k4 = deriv_feedforward(x4, p, K1, K2, nbar, d);

    return {
        x.q + h * (k1.q + 2.0 * k2.q + 2.0 * k3.q + k4.q) / 6.0,
        x.v + h * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v) / 6.0
    };
}

State3 rk4_integral_step(const State3& x, const Params& p, double h, double K1, double K2, double Ki, double d) {
    State3 k1 = deriv_integral(x, p, K1, K2, Ki, d);
    State3 x2{x.q + 0.5 * h * k1.q, x.v + 0.5 * h * k1.v, x.z + 0.5 * h * k1.z};
    State3 k2 = deriv_integral(x2, p, K1, K2, Ki, d);
    State3 x3{x.q + 0.5 * h * k2.q, x.v + 0.5 * h * k2.v, x.z + 0.5 * h * k2.z};
    State3 k3 = deriv_integral(x3, p, K1, K2, Ki, d);
    State3 x4{x.q + h * k3.q, x.v + h * k3.v, x.z + h * k3.z};
    State3 k4 = deriv_integral(x4, p, K1, K2, Ki, d);

    return {
        x.q + h * (k1.q + 2.0 * k2.q + 2.0 * k3.q + k4.q) / 6.0,
        x.v + h * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v) / 6.0,
        x.z + h * (k1.z + 2.0 * k2.z + 2.0 * k3.z + k4.z) / 6.0
    };
}

int main() {
    Params p;

    // Feedforward design with desired polynomial (s+2)(s+3)=s^2+5s+6.
    double a1 = 5.0;
    double a0 = 6.0;
    double K1 = p.m * a0 - p.k;
    double K2 = p.m * a1 - p.b;
    double nbar = p.k + K1;  // for position output and matched input channel

    // Integral design with desired polynomial (s+2)(s+3)(s+5)=s^3+10s^2+31s+30.
    double alpha2 = 10.0;
    double alpha1 = 31.0;
    double alpha0 = 30.0;
    double K1i = p.m * alpha1 - p.k;
    double K2i = p.m * alpha2 - p.b;
    double Ki = p.m * alpha0;

    std::cout << "Feedforward gains: K1=" << K1 << ", K2=" << K2 << ", Nbar=" << nbar << "\n";
    std::cout << "Integral gains: K1=" << K1i << ", K2=" << K2i << ", Ki=" << Ki << "\n";

    const double h = 0.001;
    const double tf = 8.0;
    const int steps = static_cast<int>(tf / h);

    std::ofstream csv("Chapter27_Lesson5_cpp_results.csv");
    csv << "t,y_feedforward_no_dist,y_feedforward_with_dist,y_integral_with_dist,u_integral\n";

    State2 xff0{0.0, 0.0};
    State2 xff1{0.0, 0.0};
    State3 xi{0.0, 0.0, 0.0};

    for (int i = 0; i <= steps; ++i) {
        double t = i * h;
        double ui = -K1i * xi.q - K2i * xi.v + Ki * xi.z;

        if (i % 10 == 0) {
            csv << t << "," << xff0.q << "," << xff1.q << "," << xi.q << "," << ui << "\n";
        }

        xff0 = rk4_feedforward_step(xff0, p, h, K1, K2, nbar, 0.0);
        xff1 = rk4_feedforward_step(xff1, p, h, K1, K2, nbar, p.d);
        xi = rk4_integral_step(xi, p, h, K1i, K2i, Ki, p.d);
    }

    std::cout << "Final feedforward y without disturbance: " << xff0.q << "\n";
    std::cout << "Final feedforward y with disturbance: " << xff1.q << "\n";
    std::cout << "Final integral y with disturbance: " << xi.q << "\n";
    std::cout << "CSV written to Chapter27_Lesson5_cpp_results.csv\n";
    return 0;
}
