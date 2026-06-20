// System Dynamics — Chapter 14 (Nonlinear System Dynamics)
// Lesson 1: Sources and Types of Nonlinearities in Engineering Systems
//
// Nonlinear mass–spring–damper demonstration with RK4 integration.
// Writes trajectory to CSV for plotting in any tool.
//
// Build (GCC/Clang):
//   g++ -O2 -std=c++17 Chapter14_Lesson1.cpp -o Chapter14_Lesson1
//
// Run:
//   ./Chapter14_Lesson1
//
// Output:
//   Chapter14_Lesson1_trace_cpp.csv

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

struct Params {
    double m   = 1.0;
    double c   = 0.4;
    double k   = 4.0;
    double k3  = 8.0;
    double Fc  = 0.8;
    double vs  = 0.02;
    double b   = 1.0;
    double umax= 1.5;
};

static inline double sat(double u, double umax) {
    if (u >  umax) return  umax;
    if (u < -umax) return -umax;
    return u;
}

static inline double coulomb_smooth(double v, double Fc, double vs) {
    return Fc * std::tanh(v / vs);
}

static inline double input_u(double t) {
    return 1.2 * std::sin(1.0 * t) + 0.3 * std::sin(3.0 * t);
}

struct State {
    double x;   // position
    double v;   // velocity
};

static inline State f(double t, const State& s, const Params& p) {
    double u = sat(input_u(t), p.umax);
    double spring = p.k * s.x + p.k3 * s.x * s.x * s.x;
    double fric = p.c * s.v + coulomb_smooth(s.v, p.Fc, p.vs);
    double a = (p.b * u - spring - fric) / p.m;
    return State{ s.v, a };
}

static inline State add(const State& a, const State& b) {
    return State{ a.x + b.x, a.v + b.v };
}

static inline State mul(double h, const State& a) {
    return State{ h * a.x, h * a.v };
}

int main() {
    Params p;
    const double t0 = 0.0, tf = 20.0;
    double h = 0.005;

    State s{0.4, 0.0};

    std::ofstream out("Chapter14_Lesson1_trace_cpp.csv");
    out << "t,x,xdot\n";

    double t = t0;
    while (t < tf - 1e-12) {
        out << t << "," << s.x << "," << s.v << "\n";

        if (t + h > tf) h = tf - t;

        State k1 = f(t, s, p);
        State k2 = f(t + 0.5*h, add(s, mul(0.5*h, k1)), p);
        State k3 = f(t + 0.5*h, add(s, mul(0.5*h, k2)), p);
        State k4 = f(t + h,     add(s, mul(h,     k3)), p);

        s = add(s, mul(h/6.0, add(add(k1, mul(2.0, k2)), add(mul(2.0, k3), k4))));
        t += h;
    }
    out << tf << "," << s.x << "," << s.v << "\n";

    std::cout << "Wrote: Chapter14_Lesson1_trace_cpp.csv\n";
    return 0;
}
