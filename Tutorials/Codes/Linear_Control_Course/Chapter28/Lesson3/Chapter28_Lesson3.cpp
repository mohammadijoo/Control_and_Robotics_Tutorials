#include <iostream>
#include <complex>
#include <vector>

int main() {
    double J = 0.01;
    double b = 0.1;
    double Km = 1.0;
    double K = 50.0; // loop gain

    using cdouble = std::complex<double>;

    // Frequency grid (rad/s)
    std::vector<double> omega;
    for (double w = 0.1; w <= 100.0; w *= 1.1) {
        omega.push_back(w);
    }

    std::vector<cdouble> L_vals;
    for (double w : omega) {
        cdouble s(0.0, w); // j * w
        cdouble G = Km / (s * (J * s + b));
        cdouble L = K * G;
        L_vals.push_back(L);
    }

    // Print magnitude (dB) and phase (deg) for Bode-style output
    for (std::size_t i = 0; i < omega.size(); ++i) {
        cdouble L = L_vals[i];
        double mag = std::abs(L);
        double phase = std::arg(L) * 180.0 / M_PI;
        double mag_db = 20.0 * std::log10(mag);
        std::cout << omega[i] << " " << mag_db
                  << " " << phase << std::endl;
    }

    // The resulting data can be plotted using a plotting library or exported
    // to Python/MATLAB. A Nyquist diagram uses the complex values L_vals directly.

    return 0;
}
