#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

int main() {
    const double K = 10.0;
    const double z = 10.0;
    const double wn = 20.0;
    const double zeta = 0.3;
    const double Tdelay = 0.02;

    // Frequency grid (rad/s)
    std::vector<double> omega;
    for (int k = 0; k <= 100; ++k) {
        double exp10 = 0.0 + 3.0 * k / 100.0;  // from 10^0 to 10^3
        omega.push_back(std::pow(10.0, exp10));
    }

    using cd = std::complex<double>;
    const cd j(0.0, 1.0);

    for (double w : omega) {
        cd s = j * w;

        // Real zero: 1 + s/z
        cd G_zero = 1.0 + s / z;

        // Complex pole pair: 1 / (1 + 2*zeta*s/wn + (s/wn)^2)
        cd denom = 1.0 + 2.0 * zeta * s / wn + (s / wn) * (s / wn);
        cd G_poles = 1.0 / denom;

        // Pure delay: exp(-s T)
        cd G_delay = std::exp(-s * Tdelay);

        cd G = K * G_zero * G_poles * G_delay;

        double mag = std::abs(G);
        double phase_rad = std::arg(G);
        double mag_dB = 20.0 * std::log10(mag);
        double phase_deg = phase_rad * 180.0 / M_PI;

        std::cout << w
                  << " " << mag_dB
                  << " " << phase_deg
                  << std::endl;
    }

    return 0;
}
