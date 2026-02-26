#include <vector>
#include <cmath>
#include <cstddef>
#include <iostream>

struct Metrics {
    double tr;     // 10-90% rise time
    double tp;     // peak time
    double Mp;     // maximum overshoot (relative)
    double ts;     // settling time (2% band)
    double c_inf;  // final value
};

std::vector<double> secondOrderStepResponse(
    const std::vector<double>& t,
    double zeta,
    double wn)
{
    std::vector<double> y;
    y.reserve(t.size());
    double sqrt_term = std::sqrt(1.0 - zeta * zeta);
    double wd = wn * sqrt_term;
    double phi = std::atan2(sqrt_term, zeta);
    for (double tk : t) {
        double envelope = std::exp(-zeta * wn * tk);
        double s = std::sin(wd * tk + phi);
        double ck = 1.0 - (envelope / sqrt_term) * s;
        y.push_back(ck);
    }
    return y;
}

Metrics transientMetricsFromSamples(
    const std::vector<double>& t,
    const std::vector<double>& y,
    double band = 0.02)
{
    std::size_t n = t.size();
    std::size_t n_tail = std::max<std::size_t>(1, n / 10);

    // Final value as mean of last 10% samples
    double c_inf = 0.0;
    for (std::size_t i = n - n_tail; i < n; ++i) {
        c_inf += y[i];
    }
    c_inf /= static_cast<double>(n_tail);

    // Rise time (10-90%)
    double low = 0.1 * c_inf;
    double high = 0.9 * c_inf;
    double t10 = -1.0;
    double t90 = -1.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (t10 < 0.0 && y[i] >= low) {
            t10 = t[i];
        }
        if (t90 < 0.0 && y[i] >= high) {
            t90 = t[i];
            break;
        }
    }
    double tr = (t10 >= 0.0 && t90 >= 0.0) ? (t90 - t10) : -1.0;

    // Peak time and Mp
    std::size_t idx_max = 0;
    for (std::size_t i = 1; i < n; ++i) {
        if (y[i] > y[idx_max]) {
            idx_max = i;
        }
    }
    double tp = t[idx_max];
    double Mp = (y[idx_max] - c_inf) / c_inf;

    // Settling time (2% band by default)
    double tol = band * std::fabs(c_inf);
    double ts = t.back();
    for (std::size_t k = 0; k < n; ++k) {
        bool inside = true;
        for (std::size_t j = k; j < n; ++j) {
            if (std::fabs(y[j] - c_inf) > tol) {
                inside = false;
                break;
            }
        }
        if (inside) {
            ts = t[k];
            break;
        }
    }

    Metrics m;
    m.tr = tr;
    m.tp = tp;
    m.Mp = Mp;
    m.ts = ts;
    m.c_inf = c_inf;
    return m;
}

int main() {
    // Example usage: zeta = 0.5, wn = 8 rad/s
    double zeta = 0.5;
    double wn = 8.0;

    std::vector<double> t;
    double t_end = 3.0;
    double dt = 0.001;
    for (double tk = 0.0; tk <= t_end + 1e-12; tk += dt) {
        t.push_back(tk);
    }

    std::vector<double> y = secondOrderStepResponse(t, zeta, wn);
    Metrics m = transientMetricsFromSamples(t, y, 0.02);

    std::cout << "tr = " << m.tr
              << ", tp = " << m.tp
              << ", Mp = " << m.Mp
              << ", ts = " << m.ts
              << std::endl;

    return 0;
}
