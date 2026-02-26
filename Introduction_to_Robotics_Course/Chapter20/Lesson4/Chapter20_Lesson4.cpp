#include <vector>
#include <cmath>
#include <iostream>

struct Metrics {
    double iae;
    double ise;
    double itae;
    double rmse;
    double max_abs_err;
};

Metrics compute_metrics(const std::vector<double>& t,
                        const std::vector<double>& r,
                        const std::vector<double>& y)
{
    const std::size_t N = t.size();
    if (N == 0 || r.size() != N || y.size() != N) {
        throw std::runtime_error("Input vectors must have same nonzero size.");
    }

    const double dt = (N > 1) ? (t[1] - t[0]) : 0.0;

    double iae = 0.0;
    double ise = 0.0;
    double itae = 0.0;
    double sum_e2 = 0.0;
    double max_abs_err = 0.0;

    for (std::size_t k = 0; k < N; ++k) {
        double e = r[k] - y[k];
        double ae = std::fabs(e);

        iae += ae * dt;
        ise += e * e * dt;
        itae += t[k] * ae * dt;
        sum_e2 += e * e;

        if (ae > max_abs_err) {
            max_abs_err = ae;
        }
    }

    Metrics m;
    m.iae = iae;
    m.ise = ise;
    m.itae = itae;
    m.rmse = std::sqrt(sum_e2 / static_cast<double>(N));
    m.max_abs_err = max_abs_err;
    return m;
}

int main()
{
    // Example usage with synthetic data
    std::vector<double> t;
    std::vector<double> r;
    std::vector<double> y;
    const std::size_t N = 1001;
    const double T = 10.0;
    t.reserve(N);
    r.reserve(N);
    y.reserve(N);

    for (std::size_t k = 0; k < N; ++k) {
        double tk = T * static_cast<double>(k) / static_cast<double>(N - 1);
        t.push_back(tk);
        r.push_back(1.0);
        double yk = 1.0 - std::exp(-tk) * std::cos(2.0 * tk);
        y.push_back(yk);
    }

    Metrics m = compute_metrics(t, r, y);
    std::cout << "RMSE = " << m.rmse << std::endl;
    std::cout << "max |e| = " << m.max_abs_err << std::endl;
    return 0;
}
      
