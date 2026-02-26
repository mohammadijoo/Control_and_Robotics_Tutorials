#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

using cd = std::complex<double>;

int main() {
    const double K = 10.0;
    const double T = 0.1;

    const double w_min = 0.1;
    const double w_max = 1000.0;
    const std::size_t N = 200;

    std::vector<double> w(N);
    std::vector<double> mag_dB(N);
    std::vector<double> phase_deg(N);

    const double log_w_min = std::log10(w_min);
    const double log_w_max = std::log10(w_max);

    for (std::size_t k = 0; k < N; ++k) {
        double alpha = static_cast<double>(k) / static_cast<double>(N - 1);
        double log_w = log_w_min + alpha * (log_w_max - log_w_min);
        w[k] = std::pow(10.0, log_w);

        cd s(0.0, w[k]);              // s = j w
        cd L = K / (s * (1.0 + s * T)); // L(jw)

        double mag = std::abs(L);
        double phase_rad = std::arg(L);

        mag_dB[k] = 20.0 * std::log10(mag);
        phase_deg[k] = 180.0 / M_PI * phase_rad;
    }

    // Export data (phase_deg, mag_dB) for plotting in Python, gnuplot, etc.
    for (std::size_t k = 0; k < N; ++k) {
        std::cout << phase_deg[k] << " " << mag_dB[k] << "\n";
    }

    return 0;
}
