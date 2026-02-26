#include <iostream>
#include <functional>
#include <cmath>

// Simple numerical Laplace transform approximation for real s > 0
double laplace_numeric(const std::function<double(double)>& f,
                       double s, double t_max, int N) {
    double h = t_max / static_cast<double>(N);
    double sum = 0.0;
    for (int k = 0; k <= N; ++k) {
        double t = k * h;
        double w = (k == 0 || k == N) ? 0.5 : 1.0; // trapezoidal rule weights
        sum += w * f(t) * std::exp(-s * t);
    }
    return h * sum;
}

int main() {
    // Example: f(t) = exp(-t / tau) * u(t), tau > 0
    double tau = 0.5;
    auto f = [tau](double t) {
        if (t < 0.0) return 0.0;
        return std::exp(-t / tau);
    };

    double s = 2.0;          // evaluation point in s-domain (real axis)
    double t_max = 10.0;     // truncate at t_max
    int N = 10000;           // number of integration steps

    double F_s = laplace_numeric(f, s, t_max, N);
    std::cout << "Approximate F(" << s << ") = " << F_s << std::endl;

    // For comparison, analytic value is 1 / (s + 1/tau)
    double F_exact = 1.0 / (s + 1.0 / tau);
    std::cout << "Exact      F(" << s << ") = " << F_exact << std::endl;
    return 0;
}
