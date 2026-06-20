// Chapter15_Lesson5.cpp
// Numerical Stability, Error Control, and Model Verification via Simulation
// C++17 implementation for Chapter 15, Lesson 5 (System Dynamics)

#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

using State = std::array<double, 2>;
using RHS = std::function<State(double, const State&)>;

State add(const State& a, const State& b) {
    return {a[0] + b[0], a[1] + b[1]};
}

State scale(double s, const State& a) {
    return {s * a[0], s * a[1]};
}

State rk4_step(const RHS& f, double t, const State& y, double h) {
    State k1 = f(t, y);
    State k2 = f(t + 0.5 * h, add(y, scale(0.5 * h, k1)));
    State k3 = f(t + 0.5 * h, add(y, scale(0.5 * h, k2)));
    State k4 = f(t + h, add(y, scale(h, k3)));
    State out{};
    for (int i = 0; i < 2; ++i) {
        out[i] = y[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    return out;
}

struct AdaptiveResult {
    std::vector<double> t;
    std::vector<State> y;
    int accepted = 0;
    int rejected = 0;
};

AdaptiveResult adaptive_rk4_stepdoubling(
    const RHS& f, double t0, const State& y0, double tf,
    double h0, double rtol, double atol
) {
    const int p = 4;
    double t = t0;
    State y = y0;
    double h = h0;
    const double hMin = 1e-8;
    const double hMax = 0.2;

    AdaptiveResult out;
    out.t.push_back(t);
    out.y.push_back(y);

    while (t < tf) {
        if (t + h > tf) h = tf - t;

        State yBig = rk4_step(f, t, y, h);
        State yHalf1 = rk4_step(f, t, y, 0.5 * h);
        State yHalf2 = rk4_step(f, t + 0.5 * h, yHalf1, 0.5 * h);

        State err{};
        double accum = 0.0;
        for (int i = 0; i < 2; ++i) {
            err[i] = (yHalf2[i] - yBig[i]) / (std::pow(2.0, p) - 1.0);
            double sc = atol + rtol * std::max(std::abs(yHalf2[i]), std::abs(y[i]));
            double z = err[i] / sc;
            accum += z * z;
        }
        double errNorm = std::sqrt(accum / 2.0);

        if (errNorm <= 1.0) {
            for (int i = 0; i < 2; ++i) y[i] = yHalf2[i] + err[i];
            t += h;
            out.t.push_back(t);
            out.y.push_back(y);
            out.accepted++;

            double factor = (errNorm == 0.0) ? 2.0 : 0.9 * std::pow(1.0 / errNorm, 1.0 / (p + 1.0));
            factor = std::max(0.3, std::min(2.0, factor));
            h = std::max(hMin, std::min(hMax, h * factor));
        } else {
            out.rejected++;
            double factor = 0.9 * std::pow(1.0 / std::max(errNorm, 1e-16), 1.0 / (p + 1.0));
            factor = std::max(0.1, std::min(0.5, factor));
            h = std::max(hMin, h * factor);
            if (h <= hMin) throw std::runtime_error("Step size underflow.");
        }
    }
    return out;
}

RHS damped_oscillator_rhs(double omegaN, double zeta) {
    return [omegaN, zeta](double /*t*/, const State& x) -> State {
        double q = x[0], v = x[1];
        return {v, -2.0 * zeta * omegaN * v - omegaN * omegaN * q};
    };
}

double exact_q(double t, double omegaN, double zeta, double q0, double v0) {
    double wd = omegaN * std::sqrt(1.0 - zeta * zeta);
    double A = q0;
    double B = (v0 + zeta * omegaN * q0) / wd;
    return std::exp(-zeta * omegaN * t) * (A * std::cos(wd * t) + B * std::sin(wd * t));
}

double energy(const State& x, double omegaN) {
    return 0.5 * (x[1] * x[1] + omegaN * omegaN * x[0] * x[0]);
}

void euler_stability_demo(double lambda, const std::vector<double>& hs) {
    std::cout << "=== Explicit Euler absolute-stability demo ===\n";
    for (double h : hs) {
        double amp = std::abs(1.0 + h * lambda);
        double y = 1.0;
        for (int n = 0; n < 30; ++n) y = y + h * lambda * y;
        std::cout << "h=" << std::setw(6) << h
                  << ", |1+h*lambda|=" << std::setw(10) << amp
                  << ", |y_N|=" << std::scientific << y << std::defaultfloat << "\n";
    }
    std::cout << "\n";
}

void convergence_demo() {
    std::cout << "=== RK4 convergence verification ===\n";
    double omegaN = 4.0, zeta = 0.1, tf = 5.0;
    State y0{1.0, 0.0};
    RHS f = damped_oscillator_rhs(omegaN, zeta);

    std::vector<double> hs{0.2, 0.1, 0.05, 0.025};
    std::vector<double> errs;

    for (double h : hs) {
        int n = static_cast<int>(std::round(tf / h));
        double t = 0.0;
        State y = y0;
        double maxErr = 0.0;
        for (int k = 0; k <= n; ++k) {
            double qEx = exact_q(t, omegaN, zeta, y0[0], y0[1]);
            maxErr = std::max(maxErr, std::abs(y[0] - qEx));
            if (k < n) {
                y = rk4_step(f, t, y, h);
                t += h;
            }
        }
        errs.push_back(maxErr);
        std::cout << "h=" << h << ", max|q-q_exact|=" << std::scientific << maxErr << std::defaultfloat << "\n";
    }

    std::cout << "Observed orders:\n";
    for (size_t i = 0; i + 1 < errs.size(); ++i) {
        double pObs = std::log(errs[i] / errs[i + 1]) / std::log(2.0);
        std::cout << "p_obs(" << hs[i] << "->" << hs[i + 1] << ")=" << pObs << "\n";
    }
    std::cout << "\n";
}

void adaptive_demo() {
    std::cout << "=== Adaptive RK4 (step-doubling) demo ===\n";
    double omegaN = 4.0, zeta = 0.05, tf = 12.0;
    State y0{1.0, 0.0};
    RHS f = damped_oscillator_rhs(omegaN, zeta);

    AdaptiveResult r = adaptive_rk4_stepdoubling(f, 0.0, y0, tf, 0.1, 1e-6, 1e-9);

    bool monotoneEnergy = true;
    double prevE = energy(r.y.front(), omegaN);
    for (size_t i = 1; i < r.y.size(); ++i) {
        double Ei = energy(r.y[i], omegaN);
        if (Ei - prevE > 1e-8) monotoneEnergy = false;
        prevE = Ei;
    }

    double maxErr = 0.0;
    for (size_t i = 0; i < r.t.size(); ++i) {
        double qEx = exact_q(r.t[i], omegaN, zeta, y0[0], y0[1]);
        maxErr = std::max(maxErr, std::abs(r.y[i][0] - qEx));
    }

    std::cout << "Accepted=" << r.accepted << ", Rejected=" << r.rejected << "\n";
    std::cout << "Final state q=" << r.y.back()[0] << ", v=" << r.y.back()[1] << "\n";
    std::cout << "Energy nonincreasing = " << (monotoneEnergy ? "true" : "false") << "\n";
    std::cout << "max|q-q_exact| on adaptive mesh = " << std::scientific << maxErr << std::defaultfloat << "\n";
}

int main() {
    euler_stability_demo(-50.0, {0.01, 0.03, 0.05, 0.06});
    convergence_demo();
    adaptive_demo();
    return 0;
}
