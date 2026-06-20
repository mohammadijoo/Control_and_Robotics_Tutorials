// Chapter20_Lesson2.cpp
// System Dynamics — Chapter 20 (Chaos, Complex Dynamics, and Computational Tools)
// Lesson 2: Bifurcation Diagrams, Period Doubling, and Strange Attractors
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter20_Lesson2.cpp -o ch20_l2
// Run:
//   ./ch20_l2
//
// Outputs:
//   logistic_bifurcation_cpp.csv  (r,x points)
//   lorenz_traj_cpp.csv           (t,x,y,z samples)

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

static inline double logistic(double x, double r) {
    return r * x * (1.0 - x);
}

void bifurcation_logistic(double rmin=2.5, double rmax=4.0, int Nr=6000,
                          int nTransient=1200, int nKeep=200, double x0=0.123456) {
    std::vector<double> rs(Nr), x(Nr, x0);
    for (int i = 0; i < Nr; ++i) {
        rs[i] = rmin + (rmax - rmin) * (double(i) / double(Nr - 1));
    }

    // Transient
    for (int k = 0; k < nTransient; ++k) {
        for (int i = 0; i < Nr; ++i) x[i] = logistic(x[i], rs[i]);
    }

    std::ofstream out("logistic_bifurcation_cpp.csv");
    out << "r,x\n";
    for (int k = 0; k < nKeep; ++k) {
        for (int i = 0; i < Nr; ++i) x[i] = logistic(x[i], rs[i]);
        for (int i = 0; i < Nr; ++i) out << rs[i] << "," << x[i] << "\n";
    }
    out.close();
}

struct Vec3 {
    double x, y, z;
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double a) const { return {a*x, a*y, a*z}; }
};

Vec3 lorenz_rhs(const Vec3& s, double sigma, double rho, double beta) {
    return {
        sigma*(s.y - s.x),
        s.x*(rho - s.z) - s.y,
        s.x*s.y - beta*s.z
    };
}

Vec3 rk4_step(Vec3 y, double h, double sigma, double rho, double beta) {
    Vec3 k1 = lorenz_rhs(y, sigma, rho, beta);
    Vec3 k2 = lorenz_rhs(y + k1*(0.5*h), sigma, rho, beta);
    Vec3 k3 = lorenz_rhs(y + k2*(0.5*h), sigma, rho, beta);
    Vec3 k4 = lorenz_rhs(y + k3*h, sigma, rho, beta);
    return y + (k1 + k2*2.0 + k3*2.0 + k4) * (h/6.0);
}

void simulate_lorenz(double T=40.0, double h=0.005,
                     Vec3 y0={1.0,1.0,1.0},
                     double sigma=10.0, double rho=28.0, double beta=8.0/3.0,
                     double transient=5.0, int stride=4) {
    int N = int(T / h);
    double t = 0.0;
    Vec3 y = y0;

    std::ofstream out("lorenz_traj_cpp.csv");
    out << "t,x,y,z\n";

    for (int i = 0; i < N; ++i) {
        y = rk4_step(y, h, sigma, rho, beta);
        t += h;
        if (t >= transient && (i % stride == 0)) {
            out << t << "," << y.x << "," << y.y << "," << y.z << "\n";
        }
    }
    out.close();
}

int main() {
    bifurcation_logistic();
    simulate_lorenz();
    std::cout << "Done. Wrote CSV files." << std::endl;
    return 0;
}
