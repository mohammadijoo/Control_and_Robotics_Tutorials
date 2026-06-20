// Chapter15_Lesson2.cpp
// Runge-Kutta Methods and Step Size Selection
// C++17 example: RK4 + adaptive step-doubling controller.
// Related library (not required to compile this file): Boost.Odeint.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

struct AdaptiveResult {
    std::vector<double> t;
    std::vector<double> y;
    std::vector<double> h_history;
    int accepted = 0;
    int rejected = 0;
};

double f(double t, double y) {
    return -2.0 * y + std::sin(t);
}

double y_exact(double t) {
    return (6.0 / 5.0) * std::exp(-2.0 * t) + (2.0 * std::sin(t) - std::cos(t)) / 5.0;
}

double rk4_step(double (*fun)(double, double), double t, double y, double h) {
    const double k1 = fun(t, y);
    const double k2 = fun(t + 0.5 * h, y + 0.5 * h * k1);
    const double k3 = fun(t + 0.5 * h, y + 0.5 * h * k2);
    const double k4 = fun(t + h, y + h * k3);
    return y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

std::pair<std::vector<double>, std::vector<double>> integrate_fixed_rk4(
    double t0, double tf, double y0, double h
) {
    std::vector<double> ts{t0};
    std::vector<double> ys{y0};
    double t = t0;
    double y = y0;

    while ((tf - t) > 1e-12) {
        const double h_step = std::min(h, tf - t);
        y = rk4_step(f, t, y, h_step);
        t += h_step;
        ts.push_back(t);
        ys.push_back(y);
    }
    return {ts, ys};
}

AdaptiveResult integrate_adaptive_rk4_stepdoubling(
    double t0, double tf, double y0,
    double h0 = 0.2, double atol = 1e-8, double rtol = 1e-6,
    double h_min = 1e-8, double h_max = 0.5, double safety = 0.9
) {
    AdaptiveResult out;
    out.t.push_back(t0);
    out.y.push_back(y0);

    double t = t0;
    double y = y0;
    double h = h0;

    while ((tf - t) > 1e-12) {
        h = std::min(h, tf - t);
        if (h < h_min) {
            throw std::runtime_error("Step size below h_min");
        }

        const double y_full = rk4_step(f, t, y, h);
        const double y_half = rk4_step(f, t, y, 0.5 * h);
        const double y_half2 = rk4_step(f, t + 0.5 * h, y_half, 0.5 * h);

        const double err_est = std::abs(y_half2 - y_full) / 15.0;
        const double scale = atol + rtol * std::max(std::abs(y), std::abs(y_half2));
        const double err_norm = (scale > 0.0) ? (err_est / scale) : err_est;

        if (err_norm <= 1.0) {
            y = y_half2 + (y_half2 - y_full) / 15.0;
            t += h;
            out.t.push_back(t);
            out.y.push_back(y);
            out.h_history.push_back(h);
            out.accepted++;

            double factor;
            if (err_norm == 0.0) {
                factor = 2.0;
            } else {
                factor = safety * std::pow(1.0 / err_norm, 1.0 / 5.0);
            }
            factor = std::min(2.0, std::max(0.2, factor));
            h = std::min(h_max, factor * h);
        } else {
            out.rejected++;
            double factor = safety * std::pow(1.0 / std::max(err_norm, 1e-16), 1.0 / 5.0);
            factor = std::min(1.0, std::max(0.1, factor));
            h = std::max(h_min, factor * h);
        }
    }

    return out;
}

double max_error(const std::vector<double>& ts, const std::vector<double>& ys) {
    double emax = 0.0;
    for (std::size_t i = 0; i < ts.size(); ++i) {
        emax = std::max(emax, std::abs(ys[i] - y_exact(ts[i])));
    }
    return emax;
}

int main() {
    const double t0 = 0.0, tf = 10.0, y0 = 1.0;

    auto [t_fixed, y_fixed] = integrate_fixed_rk4(t0, tf, y0, 0.1);
    AdaptiveResult adaptive = integrate_adaptive_rk4_stepdoubling(t0, tf, y0);

    std::cout << std::scientific << std::setprecision(6);
    std::cout << "Chapter15_Lesson2.cpp\n";
    std::cout << "Fixed RK4 (h=0.1): steps = " << (t_fixed.size() - 1)
              << ", max error = " << max_error(t_fixed, y_fixed) << "\n";

    std::cout << "Adaptive RK4 step-doubling: accepted = " << adaptive.accepted
              << ", rejected = " << adaptive.rejected
              << ", max error = " << max_error(adaptive.t, adaptive.y) << "\n";

    if (!adaptive.h_history.empty()) {
        double hmin = adaptive.h_history.front();
        double hmax = adaptive.h_history.front();
        for (double h : adaptive.h_history) {
            hmin = std::min(hmin, h);
            hmax = std::max(hmax, h);
        }
        std::cout << "h range = [" << hmin << ", " << hmax << "]\n";
    }

    std::cout << "\nLibrary note: Boost.Odeint offers explicit_rk4, runge_kutta_cash_karp54, "
                 "and controlled steppers for adaptive integration.\n";
    return 0;
}
