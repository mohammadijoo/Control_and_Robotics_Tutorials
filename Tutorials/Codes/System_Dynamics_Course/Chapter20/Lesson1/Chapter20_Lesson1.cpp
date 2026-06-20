// Chapter20_Lesson1.cpp
/*
System Dynamics — Chapter 20, Lesson 1
Nonlinear Maps and Continuous-Time Chaotic Systems (Logistic Map, Lorenz System)

This program:
1) Simulates the logistic map and writes x_n to CSV.
2) Integrates the Lorenz system using classical RK4 and writes trajectory to CSV.

Build (example):
  g++ -O2 -std=c++17 Chapter20_Lesson1.cpp -o ch20_l1

Outputs:
  logistic.csv  (n, x_n)
  lorenz.csv    (t, x, y, z)
*/

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

static std::vector<double> logistic_map(double r, double x0, int n) {
    std::vector<double> x(n + 1);
    x[0] = x0;
    for (int k = 0; k < n; ++k) {
        x[k + 1] = r * x[k] * (1.0 - x[k]);
    }
    return x;
}

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

static Vec3 operator+(const Vec3& a, const Vec3& b) { return Vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
static Vec3 operator-(const Vec3& a, const Vec3& b) { return Vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
static Vec3 operator*(double s, const Vec3& a) { return Vec3(s * a.x, s * a.y, s * a.z); }
static Vec3 operator*(const Vec3& a, double s) { return s * a; }

static Vec3 lorenz_rhs(double /*t*/, const Vec3& s, double sigma, double rho, double beta) {
    Vec3 ds;
    ds.x = sigma * (s.y - s.x);
    ds.y = s.x * (rho - s.z) - s.y;
    ds.z = s.x * s.y - beta * s.z;
    return ds;
}

static Vec3 rk4_step(double t, const Vec3& x, double h, double sigma, double rho, double beta) {
    Vec3 k1 = lorenz_rhs(t, x, sigma, rho, beta);
    Vec3 k2 = lorenz_rhs(t + 0.5 * h, x + 0.5 * h * k1, sigma, rho, beta);
    Vec3 k3 = lorenz_rhs(t + 0.5 * h, x + 0.5 * h * k2, sigma, rho, beta);
    Vec3 k4 = lorenz_rhs(t + h, x + h * k3, sigma, rho, beta);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

int main() {
    // Logistic map
    const double r = 3.8;
    const double x0 = 0.2;
    const int n = 2000;
    auto x = logistic_map(r, x0, n);

    std::ofstream flog("logistic.csv");
    flog << "n,x\n";
    for (int k = 0; k <= n; ++k) {
        flog << k << "," << x[k] << "\n";
    }
    flog.close();

    // Lorenz RK4
    const double sigma = 10.0;
    const double rho = 28.0;
    const double beta = 8.0 / 3.0;
    const double t0 = 0.0;
    const double tf = 40.0;
    const double h = 0.005;

    int steps = static_cast<int>(std::ceil((tf - t0) / h));
    std::ofstream flor("lorenz.csv");
    flor << "t,x,y,z\n";

    double t = t0;
    Vec3 s(1.0, 1.0, 1.0);
    flor << t << "," << s.x << "," << s.y << "," << s.z << "\n";
    for (int k = 0; k < steps; ++k) {
        s = rk4_step(t, s, h, sigma, rho, beta);
        t += h;
        flor << t << "," << s.x << "," << s.y << "," << s.z << "\n";
    }
    flor.close();

    std::cout << "Wrote logistic.csv and lorenz.csv\n";
    return 0;
}
