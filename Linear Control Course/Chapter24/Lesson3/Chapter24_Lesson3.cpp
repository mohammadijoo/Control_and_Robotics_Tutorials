#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

int main() {
    using std::complex;
    using std::vector;

    const double J = 0.02;
    const double B = 0.1;
    const double K = 1.0;

    const double kp = 50.0;
    const double Ti = 0.1;
    const double Tl = 0.02;
    const double alpha = 0.2;

    const complex<double> j(0.0, 1.0);

    auto P = [&] (double w) {
        complex<double> s = j * w;
        return K / (J * s * s + B * s);
    };

    auto C = [&] (double w) {
        complex<double> s = j * w;
        complex<double> pi = (1.0 + 1.0 / (Ti * s));
        complex<double> lead = (Tl * s + 1.0) / (alpha * Tl * s + 1.0);
        return kp * pi * lead;
    };

    auto W2 = [&] (double w) {
        complex<double> s = j * w;
        return 0.2 * (s / 50.0 + 1.0) / (s / 500.0 + 1.0);
    };

    // Logarithmic frequency grid
    vector<double> w;
    for (int k = 0; k <= 600; ++k) {
        double exponent = -1.0 + 4.0 * static_cast<double>(k) / 600.0;
        w.push_back(std::pow(10.0, exponent));
    }

    double MS = 0.0;
    double robust_index = 0.0;

    for (std::size_t k = 0; k < w.size(); ++k) {
        double wk = w[k];
        complex<double> L = C(wk) * P(wk);
        complex<double> S = 1.0 / (1.0 + L);
        complex<double> T = L / (1.0 + L);

        double magS = std::abs(S);
        double magWT = std::abs(W2(wk) * T);

        if (magS > MS) {
            MS = magS;
        }
        if (magWT > robust_index) {
            robust_index = magWT;
        }
    }

    std::cout << "Maximum sensitivity M_S ≈ " << MS << std::endl;
    std::cout << "max |W2(jw) T(jw)| ≈ " << robust_index << std::endl;

    if (robust_index < 1.0) {
        std::cout << "Small-gain robust stability condition satisfied." << std::endl;
    } else {
        std::cout << "Robust stability not guaranteed." << std::endl;
    }

    return 0;
}
