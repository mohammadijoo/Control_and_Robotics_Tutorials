/*
Chapter 14 - Nonlinear System Dynamics
Lesson 3 - Linearization vs. True Nonlinear Behavior: When Linear Models Fail

File: Chapter14_Lesson3.cpp
Build (example, g++):
  g++ -O2 -std=c++17 Chapter14_Lesson3.cpp -o Chapter14_Lesson3

This program simulates Example B:
  xdot = -y + x (x^2 + y^2)
  ydot =  x + y (x^2 + y^2)
and its linearization at the origin:
  zdot = A z,  A = [[0,-1],[1,0]]
Outputs CSV: Chapter14_Lesson3_ExampleB_cpp.csv
*/

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

static Vec2 add(const Vec2& a, const Vec2& b) { return {a[0] + b[0], a[1] + b[1]}; }
static Vec2 sub(const Vec2& a, const Vec2& b) { return {a[0] - b[0], a[1] - b[1]}; }
static Vec2 mul(double s, const Vec2& a) { return {s * a[0], s * a[1]}; }
static double norm2(const Vec2& a) { return std::sqrt(a[0] * a[0] + a[1] * a[1]); }

static Vec2 matvec(const Mat2& A, const Vec2& x) {
    return {A[0][0] * x[0] + A[0][1] * x[1],
            A[1][0] * x[0] + A[1][1] * x[1]};
}

static Vec2 f_nl(double /*t*/, const Vec2& z) {
    const double x = z[0], y = z[1];
    const double r2 = x * x + y * y;
    return {-y + x * r2, x + y * r2};
}

static Vec2 f_lin(double /*t*/, const Vec2& z) {
    static const Mat2 A = {{{0.0, -1.0}, {1.0, 0.0}}};
    return matvec(A, z);
}

template <typename F>
static Vec2 rk4_step(F f, double t, const Vec2& x, double h) {
    const Vec2 k1 = f(t, x);
    const Vec2 k2 = f(t + 0.5 * h, add(x, mul(0.5 * h, k1)));
    const Vec2 k3 = f(t + 0.5 * h, add(x, mul(0.5 * h, k2)));
    const Vec2 k4 = f(t + h, add(x, mul(h, k3)));
    Vec2 sum = add(k1, add(mul(2.0, k2), add(mul(2.0, k3), k4)));
    return add(x, mul(h / 6.0, sum));
}

int main() {
    const double t0 = 0.0, tf = 25.0, h = 1e-3;
    const int nSteps = static_cast<int>(std::ceil((tf - t0) / h));

    Vec2 z0 = {0.2, 0.0};

    double t = t0;
    Vec2 z_nl = z0;
    Vec2 z_lin = z0; // perturbation coords = state here (equilibrium at origin)

    std::ofstream ofs("Chapter14_Lesson3_ExampleB_cpp.csv");
    ofs << "t,x_lin,y_lin,r_lin,x_nl,y_nl,r_nl\n";

    for (int k = 0; k <= nSteps; ++k) {
        const double rlin = norm2(z_lin);
        const double rnl  = norm2(z_nl);
        ofs << t << "," << z_lin[0] << "," << z_lin[1] << "," << rlin << ","
            << z_nl[0] << "," << z_nl[1] << "," << rnl << "\n";

        if (k == nSteps) break;
        z_lin = rk4_step(f_lin, t, z_lin, h);
        z_nl  = rk4_step(f_nl,  t, z_nl,  h);
        t += h;
    }

    ofs.close();
    std::cout << "Saved: Chapter14_Lesson3_ExampleB_cpp.csv\n";
    std::cout << "Final r_lin=" << norm2(z_lin) << "  r_nl=" << norm2(z_nl) << "\n";
    return 0;
}
