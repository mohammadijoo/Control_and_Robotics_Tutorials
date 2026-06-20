// Chapter20_Lesson3.cpp
// System Dynamics (Control Engineering) - Chapter 20, Lesson 3
// Sensitivity to Initial Conditions and Lyapunov Exponents (Intro)

#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>
#include <random>

// -----------------------------
// Part A) Logistic map LLE
// x_{n+1} = r x_n (1 - x_n)
// lambda ≈ (1/N) sum log |r (1 - 2 x_n)|
// -----------------------------
static inline double logistic_step(double x, double r) {
    return r * x * (1.0 - x);
}

double lyapunov_logistic(double r, double x0=0.2, int n=200000, int discard=5000) {
    double x = x0;
    for (int i=0; i<discard; ++i) x = logistic_step(x, r);

    long double s = 0.0L;
    for (int i=0; i<n; ++i) {
        double fp = r * (1.0 - 2.0*x);
        s += std::log(std::fabs(fp) + 1e-300);
        x = logistic_step(x, r);
    }
    return (double)(s / (long double)n);
}

// -----------------------------
// Part B) Lorenz LLE (two-trajectory renormalization) with RK4
// -----------------------------
using Vec3 = std::array<double,3>;

static inline Vec3 lorenz_rhs(const Vec3& X, double sigma=10.0, double rho=28.0, double beta=8.0/3.0) {
    const double x = X[0], y = X[1], z = X[2];
    return Vec3{
        sigma * (y - x),
        x * (rho - z) - y,
        x * y - beta * z
    };
}

static inline Vec3 add(const Vec3& a, const Vec3& b) {
    return Vec3{a[0]+b[0], a[1]+b[1], a[2]+b[2]};
}
static inline Vec3 scale(const Vec3& a, double s) {
    return Vec3{a[0]*s, a[1]*s, a[2]*s};
}
static inline Vec3 sub(const Vec3& a, const Vec3& b) {
    return Vec3{a[0]-b[0], a[1]-b[1], a[2]-b[2]};
}
static inline double norm2(const Vec3& a) {
    return std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

Vec3 rk4_step_lorenz(const Vec3& X, double dt, double sigma=10.0, double rho=28.0, double beta=8.0/3.0) {
    const Vec3 k1 = lorenz_rhs(X, sigma, rho, beta);
    const Vec3 k2 = lorenz_rhs(add(X, scale(k1, 0.5*dt)), sigma, rho, beta);
    const Vec3 k3 = lorenz_rhs(add(X, scale(k2, 0.5*dt)), sigma, rho, beta);
    const Vec3 k4 = lorenz_rhs(add(X, scale(k3, dt)), sigma, rho, beta);
    return add(X, scale(add(add(k1, scale(k2,2.0)), add(scale(k3,2.0), k4)), dt/6.0));
}

double lyapunov_lorenz_lle(
    Vec3 X0 = {1.0,1.0,1.0},
    double dt = 0.01,
    double T = 100.0,
    double transient = 10.0,
    int renorm_every = 10,
    double d0 = 1e-8,
    double sigma = 10.0,
    double rho = 28.0,
    double beta = 8.0/3.0,
    unsigned seed = 0
) {
    std::mt19937 gen(seed);
    std::normal_distribution<double> N01(0.0, 1.0);

    Vec3 X = X0;
    Vec3 u{N01(gen), N01(gen), N01(gen)};
    double un = norm2(u);
    u = scale(u, 1.0/un);
    Vec3 Xp = add(X, scale(u, d0));

    // transient
    int n_trans = (int)std::round(transient/dt);
    for (int i=0; i<n_trans; ++i) {
        X = rk4_step_lorenz(X, dt, sigma, rho, beta);
        Xp = rk4_step_lorenz(Xp, dt, sigma, rho, beta);
        Vec3 dvec = sub(Xp, X);
        double d = norm2(dvec);
        if (d == 0.0) {
            Xp = add(X, scale(u, d0));
        } else {
            Xp = add(X, scale(dvec, d0/d));
        }
    }

    // accumulate
    long double s = 0.0L;
    int steps = (int)std::round(T/dt);
    int count = 0;
    for (int k=0; k<steps; ++k) {
        X  = rk4_step_lorenz(X, dt, sigma, rho, beta);
        Xp = rk4_step_lorenz(Xp, dt, sigma, rho, beta);

        if ((k+1) % renorm_every == 0) {
            Vec3 dvec = sub(Xp, X);
            double d = norm2(dvec);
            if (d == 0.0) continue;
            s += std::log(d/d0);
            count += 1;
            Xp = add(X, scale(dvec, d0/d));
        }
    }

    double total_time = (double)count * (double)renorm_every * dt;
    return (double)(s / (long double)total_time);
}

int main() {
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "Logistic map LLE examples:\n";
    for (double r : {3.2, 3.5, 3.9, 4.0}) {
        double lam = lyapunov_logistic(r, 0.234, 100000, 5000);
        std::cout << "  r=" << r << " : lambda ~= " << lam << "\n";
    }

    std::cout << "\nLorenz LLE example (sigma=10, rho=28, beta=8/3):\n";
    double lamL = lyapunov_lorenz_lle({1.0,1.0,1.0}, 0.01, 120.0, 20.0, 10, 1e-8, 10.0, 28.0, 8.0/3.0, 0);
    std::cout << "  lambda_max ~= " << lamL << " 1/time-unit\n";

    return 0;
}
