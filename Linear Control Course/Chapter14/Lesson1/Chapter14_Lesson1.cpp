#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

// Convert amplitude ratio to dB
double ampToDb(double amp) {
    return 20.0 * std::log10(amp);
}

// First-order magnitude |G(jw)| for G(s) = K / (tau*s + 1)
double firstOrderMag(double K, double tau, double w) {
    std::complex<double> jw(0.0, w);
    std::complex<double> G = K / (1.0 + tau * jw);
    return std::abs(G);
}

int main() {
    double K   = 10.0;   // servo gain
    double tau = 0.05;   // time constant

    std::vector<double> w_values = {0.1, 1.0, 10.0, 100.0};
    for (double w : w_values) {
        double mag = firstOrderMag(K, tau, w);
        double db  = ampToDb(mag);
        std::cout << "w = " << w
                  << " rad/s, |G(jw)| = " << mag
                  << ", magnitude = " << db << " dB\n";
    }
    return 0;
}
