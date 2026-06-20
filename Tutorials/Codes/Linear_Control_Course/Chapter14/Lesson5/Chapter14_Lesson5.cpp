#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

int main() {
    using std::complex;
    using std::cout;
    using std::endl;

    double wn = 10.0;   // natural frequency
    double zeta = 0.4;  // damping ratio
    double K = 2.0;     // proportional gain

    // Frequencies for Bode computation (logspace)
    std::vector<double> omega;
    int N = 200;
    double wmin = 0.1, wmax = 100.0;
    double logwmin = std::log10(wmin), logwmax = std::log10(wmax);
    for (int i = 0; i < N; ++i) {
        double lw = logwmin + (logwmax - logwmin) * i / (N - 1);
        omega.push_back(std::pow(10.0, lw));
    }

    complex<double> j(0.0, 1.0);

    double pm_deg = 0.0;
    double w_gc = 0.0;
    bool found_gc = false;

    cout << "omega, mag_dB, phase_deg" << endl;

    for (double w : omega) {
        // s = j*w
        complex<double> s = j * w;

        // Plant G(s) = wn^2 / (s^2 + 2*zeta*wn s + wn^2)
        complex<double> numG(wn * wn, 0.0);
        complex<double> denG = s * s + 2.0 * zeta * wn * s
                                 + complex<double>(wn * wn, 0.0);
        complex<double> G = numG / denG;

        // Controller C(s) = K
        complex<double> C(K, 0.0);

        complex<double> L = C * G;

        double mag = std::abs(L);
        double mag_dB = 20.0 * std::log10(mag);
        double phase_rad = std::arg(L);
        double phase_deg = phase_rad * 180.0 / M_PI;

        cout << w << ", " << mag_dB << ", "
             << phase_deg << endl;

        // Rough gain crossover detection: |L| ~ 1 (0 dB)
        if (!found_gc && std::abs(mag - 1.0) < 0.02) {
            found_gc = true;
            w_gc = w;
            pm_deg = 180.0 + phase_deg;
        }
    }

    if (found_gc) {
        std::cout << "Approx gain crossover w_gc = "
                  << w_gc << " rad/s" << std::endl;
        std::cout << "Approx phase margin PM = "
                  << pm_deg << " deg" << std::endl;
    } else {
        std::cout << "Gain crossover not found in scanned range."
                  << std::endl;
    }

    return 0;
}
