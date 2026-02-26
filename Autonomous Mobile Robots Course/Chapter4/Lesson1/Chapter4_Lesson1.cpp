/*
Chapter4_Lesson1.cpp
Autonomous Mobile Robots - Chapter 4 (Mobile Robot Dynamics Applied)
Lesson 1: When Dynamics Matter for AMR

A minimal RK4 simulation comparing:
  (1) Kinematic unicycle driven by (v_cmd, w_cmd)
  (2) Dynamic model with bounded wheel torques producing (v_dot, w_dot)

Build:
  g++ -O2 -std=c++17 Chapter4_Lesson1.cpp -o demo

Run:
  ./demo

Author: Course materials generator
*/

#include <iostream>
#include <array>
#include <cmath>
#include <algorithm>

static inline double wrap_pi(double a) {
    const double pi = 3.14159265358979323846;
    a = std::fmod(a + pi, 2.0*pi);
    if (a < 0.0) a += 2.0*pi;
    return a - pi;
}

static inline double sat(double x, double lo, double hi) {
    return std::min(std::max(x, lo), hi);
}

struct Params {
    double m   = 30.0;
    double Iz  = 1.2;
    double rw  = 0.10;
    double b   = 0.50;
    double bv  = 6.0;
    double bw  = 0.25;
    double tau_max = 3.0;
    double kv  = 8.0;
    double kw  = 1.2;
};

static std::array<double,2> cmd_profile(double t) {
    double v_cmd, w_cmd;
    if (t <= 5.0) {
        v_cmd = 0.2 + 0.18*t;
        w_cmd = 0.6;
    } else {
        v_cmd = 1.1;
        w_cmd = 1.5;
    }
    return {v_cmd, w_cmd};
}

static std::array<double,3> kin_rhs(const std::array<double,3>& x, const std::array<double,2>& u) {
    const double px = x[0], py = x[1], th = x[2];
    const double v = u[0], w = u[1];
    return { v*std::cos(th), v*std::sin(th), w };
}

static std::array<double,5> dyn_rhs(const std::array<double,5>& x, const std::array<double,2>& u, const Params& p) {
    const double px = x[0], py = x[1], th = x[2], v = x[3], w = x[4];
    const double v_ref = u[0], w_ref = u[1];

    const double tau_sum  = p.kv*(v_ref - v);
    const double tau_diff = p.kw*(w_ref - w);

    double tau_R = 0.5*(tau_sum + tau_diff);
    double tau_L = 0.5*(tau_sum - tau_diff);

    tau_R = sat(tau_R, -p.tau_max, p.tau_max);
    tau_L = sat(tau_L, -p.tau_max, p.tau_max);

    const double Fx = (tau_R + tau_L)/p.rw;
    const double Mz = (p.b/(2.0*p.rw))*(tau_R - tau_L);

    const double v_dot = (Fx - p.bv*v)/p.m;
    const double w_dot = (Mz - p.bw*w)/p.Iz;

    return {
        v*std::cos(th),
        v*std::sin(th),
        w,
        v_dot,
        w_dot
    };
}

template <size_t N, typename RHS, typename U>
static std::array<double,N> rk4_step(RHS rhs, double dt, const std::array<double,N>& x, const U& u) {
    auto k1 = rhs(x, u);
    std::array<double,N> x2;
    for (size_t i=0;i<N;i++) x2[i] = x[i] + 0.5*dt*k1[i];
    auto k2 = rhs(x2, u);
    std::array<double,N> x3;
    for (size_t i=0;i<N;i++) x3[i] = x[i] + 0.5*dt*k2[i];
    auto k3 = rhs(x3, u);
    std::array<double,N> x4;
    for (size_t i=0;i<N;i++) x4[i] = x[i] + dt*k3[i];
    auto k4 = rhs(x4, u);

    std::array<double,N> xn;
    for (size_t i=0;i<N;i++) xn[i] = x[i] + (dt/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    return xn;
}

int main() {
    const double dt = 0.002;
    const double T  = 10.0;
    const int steps = static_cast<int>(T/dt) + 1;

    std::array<double,3> xk = {0.0, 0.0, 0.0};
    std::array<double,5> xd = {0.0, 0.0, 0.0, 0.0, 0.0};
    Params p;

    // Print a few samples so you can plot externally (CSV-like)
    std::cout << "t,xk,yk,thk,xd,yd,thd,vd,wd,v_cmd,w_cmd\n";

    for (int i=0;i<steps;i++) {
        const double t = i*dt;
        auto u = cmd_profile(t);

        if (i % 250 == 0) { // every 0.5 s
            std::cout << t << ","
                      << xk[0] << "," << xk[1] << "," << xk[2] << ","
                      << xd[0] << "," << xd[1] << "," << xd[2] << ","
                      << xd[3] << "," << xd[4] << ","
                      << u[0] << "," << u[1] << "\n";
        }

        // Integrate kinematic
        xk = rk4_step<3>(
            [](const std::array<double,3>& x, const std::array<double,2>& u){ return kin_rhs(x,u); },
            dt, xk, u
        );
        xk[2] = wrap_pi(xk[2]);

        // Integrate dynamic
        xd = rk4_step<5>(
            [&p](const std::array<double,5>& x, const std::array<double,2>& u){ return dyn_rhs(x,u,p); },
            dt, xd, u
        );
        xd[2] = wrap_pi(xd[2]);
    }

    std::cerr << "Done. Save stdout to a file to plot.\n";
    return 0;
}
