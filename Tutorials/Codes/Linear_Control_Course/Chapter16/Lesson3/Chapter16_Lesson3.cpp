#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

// Simple transfer G(s) = 1 / (s (s + 1)) evaluated at s = j w
std::complex<double> G_of_jw(double w) {
    std::complex<double> jw(0.0, w);
    return 1.0 / (jw * (jw + 1.0));
}

// First-order compensator K(s) = Kc (tau_z s + 1)/(tau_p s + 1)
std::complex<double> K_of_jw(double w, double Kc, double tau_z, double tau_p) {
    std::complex<double> jw(0.0, w);
    return Kc * (tau_z * jw + 1.0) / (tau_p * jw + 1.0);
}

int main() {
    double Kc = 4.0;
    double tau_z = 0.5;
    double tau_p = 0.05;

    std::vector<double> w_grid;
    for (int k = 0; k <= 400; ++k) {
        double w = std::pow(10.0, -2.0 + 4.0 * k / 400.0); // 10^-2 ... 10^2
        w_grid.push_back(w);
    }

    for (double w : w_grid) {
        std::complex<double> L = K_of_jw(w, Kc, tau_z, tau_p) * G_of_jw(w);
        double mag = std::abs(L);
        double phase = std::atan2(L.imag(), L.real()) * 180.0 / M_PI;
        double mag_dB = 20.0 * std::log10(mag);
        std::cout << w << " " << mag_dB << " " << phase << std::endl;
    }

    return 0;
}
