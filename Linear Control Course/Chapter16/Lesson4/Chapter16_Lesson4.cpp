#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

int main() {
    using std::complex;
    using std::vector;

    const double J = 0.01;
    const double B = 0.1;
    const double Kp = 20.0;
    const double Kd = 1.0;

    auto G = [J, B](double w) {
        complex<double> s(0.0, w);
        return 1.0 / (J * s * s + B * s);
    };

    auto C = [Kp, Kd](double w) {
        complex<double> s(0.0, w);
        return Kd * s + Kp;
    };

    // Logarithmic frequency grid: 0.1 to 100 rad/s
    vector<double> omega;
    const int N = 400;
    const double logw_min = std::log10(0.1);
    const double logw_max = std::log10(100.0);
    for (int k = 0; k < N; ++k) {
        double alpha = static_cast<double>(k) / static_cast<double>(N - 1);
        double logw = logw_min + alpha * (logw_max - logw_min);
        omega.push_back(std::pow(10.0, logw));
    }

    double Ms = 0.0;

    for (std::size_t k = 0; k < omega.size(); ++k) {
        double w = omega[k];
        complex<double> L = C(w) * G(w);
        complex<double> S = 1.0 / (1.0 + L);
        double magS = std::abs(S);
        if (magS > Ms) {
            Ms = magS;
        }

        // Export Nichols data: phase (deg), magnitude (dB)
        double phase_deg = std::arg(L) * 180.0 / M_PI;
        double mag_db = 20.0 * std::log10(std::abs(L));
        std::cout << phase_deg << " " << mag_db << std::endl;
    }

    std::cerr << "Peak sensitivity Ms = " << Ms << std::endl;
    return 0;
}
