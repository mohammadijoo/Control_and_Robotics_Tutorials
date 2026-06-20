#include <iostream>
#include <vector>
#include <cmath>

int main() {
    double dt = 0.001;
    int N = 5000;
    double tau = 0.2; // time constant

    std::vector<double> g(N), u(N), y(N, 0.0);

    // Impulse response and unit step input
    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        g[k] = (t >= 0.0) ? (1.0 / tau) * std::exp(-t / tau) : 0.0;
        u[k] = (t >= 0.0) ? 1.0 : 0.0;
    }

    // Discrete-time convolution y[k] ~= sum_j g[j] * u[k-j] * dt
    for (int k = 0; k < N; ++k) {
        double sum = 0.0;
        for (int j = 0; j <= k; ++j) {
            sum += g[j] * u[k - j] * dt;
        }
        y[k] = sum;
    }

    // Print a few samples
    for (int k = 0; k < N; k += 500) {
        double t = k * dt;
        std::cout << "t=" << t
                  << "  y(t)~" << y[k]
                  << "  analytic=" << (1.0 - std::exp(-t / tau))
                  << std::endl;
    }
    return 0;
}
