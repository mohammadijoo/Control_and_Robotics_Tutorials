/*
Chapter14_Lesson4.cpp
System Dynamics (Control Engineering) — Chapter 14, Lesson 4
Piecewise-Linear Approximations, Saturation, Dead-Zone, and Backlash Models

Build (example, MSYS2/MinGW64):
  g++ -O2 -std=c++17 Chapter14_Lesson4.cpp -o Chapter14_Lesson4.exe

Run:
  ./Chapter14_Lesson4.exe

It writes: Chapter14_Lesson4_output.csv
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <string>

static double sat(double u, double umax) {
    if (u > umax) return umax;
    if (u < -umax) return -umax;
    return u;
}

static double dead_zone(double u, double d) {
    if (std::abs(u) <= d) return 0.0;
    return u - d * ((u > 0.0) ? 1.0 : -1.0);
}

struct Backlash {
    double b;
    double y;
    explicit Backlash(double width, double y0=0.0) : b(width), y(y0) {}

    double step(double u) {
        const double half = 0.5 * b;
        if (u > y + half) y = u - half;
        else if (u < y - half) y = u + half;
        return y;
    }
};

struct PWL {
    std::vector<double> xs;
    std::vector<double> ys;

    PWL(std::vector<double> x, std::vector<double> y) : xs(std::move(x)), ys(std::move(y)) {
        if (xs.size() != ys.size() || xs.size() < 2) throw std::runtime_error("PWL needs >=2 points.");
        for (size_t i=0; i+1<xs.size(); ++i) {
            if (!(xs[i] < xs[i+1])) throw std::runtime_error("PWL xs must be strictly increasing.");
        }
    }

    double eval(double x) const {
        if (x <= xs.front()) return ys.front();
        if (x >= xs.back())  return ys.back();
        for (size_t i=0; i+1<xs.size(); ++i) {
            if (xs[i] <= x && x <= xs[i+1]) {
                const double x0 = xs[i], x1 = xs[i+1];
                const double y0 = ys[i], y1 = ys[i+1];
                const double t = (x - x0) / (x1 - x0);
                return (1.0 - t)*y0 + t*y1;
            }
        }
        return ys.back();
    }
};

static void plant(double /*t*/, const double x1, const double x2, const double u,
                  double& dx1, double& dx2, double wn=5.0, double zeta=0.25, double b=1.0) {
    dx1 = x2;
    dx2 = -2.0*zeta*wn*x2 - (wn*wn)*x1 + b*u;
}

static void rk4_step(double t, double& x1, double& x2, double dt, double u) {
    double k1_1, k1_2; plant(t, x1, x2, u, k1_1, k1_2);
    double k2_1, k2_2; plant(t+0.5*dt, x1+0.5*dt*k1_1, x2+0.5*dt*k1_2, u, k2_1, k2_2);
    double k3_1, k3_2; plant(t+0.5*dt, x1+0.5*dt*k2_1, x2+0.5*dt*k2_2, u, k3_1, k3_2);
    double k4_1, k4_2; plant(t+dt, x1+dt*k3_1, x2+dt*k3_2, u, k4_1, k4_2);

    x1 += (dt/6.0)*(k1_1 + 2.0*k2_1 + 2.0*k3_1 + k4_1);
    x2 += (dt/6.0)*(k1_2 + 2.0*k2_2 + 2.0*k3_2 + k4_2);
}

int main() {
    const double T = 6.0;
    const double dt = 1e-3;

    // PD controller
    const double Kp = 18.0, Kd = 4.5;

    // Nonlinearities
    const double u_max = 2.0;
    const double d = 0.25;
    Backlash backlash(0.20, 0.0);

    // Optional PWL map approximating tanh(u)
    std::vector<double> xs = {-3.0,-1.5,-0.5,0.0,0.5,1.5,3.0};
    std::vector<double> ys; ys.reserve(xs.size());
    for (double x : xs) ys.push_back(std::tanh(x));
    PWL pwl_tanh(xs, ys);

    const bool use_pwl_map = false;

    double x1 = 0.0, x2 = 0.0;

    std::ofstream out("Chapter14_Lesson4_output.csv");
    out << "t,x1,x2,u_cmd,u_deadzone,u_sat,u_backlash\n";
    out << std::fixed << std::setprecision(9);

    for (double t=0.0; t <= T + 1e-12; t += dt) {
        const double r = 1.0;               // step reference
        const double e = r - x1;
        const double u_cmd = Kp*e - Kd*x2;

        const double u_dz  = dead_zone(u_cmd, d);
        const double u_sat = sat(u_dz, u_max);
        const double u_map = use_pwl_map ? pwl_tanh.eval(u_sat) : u_sat;
        const double u_bl  = backlash.step(u_map);

        rk4_step(t, x1, x2, dt, u_bl);

        out << t << "," << x1 << "," << x2 << ","
            << u_cmd << "," << u_dz << "," << u_sat << "," << u_bl << "\n";
    }

    std::cout << "Wrote Chapter14_Lesson4_output.csv\n";
    std::cout << "Plot x1(t) and compare u_cmd vs u_backlash to see nonlinear effects.\n";
    return 0;
}
