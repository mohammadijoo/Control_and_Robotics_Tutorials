#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

struct RealFactorBode {
    double K;
    int n_int;                    // integrators (1/s)
    int n_diff;                   // differentiators (s)
    std::vector<double> zeros;   // w_z
    std::vector<double> poles;   // w_p

    std::complex<double> G(std::complex<double> s) const {
        std::complex<double> val = K;
        // differentiators
        for (int i = 0; i < n_diff; ++i)
            val *= s;
        // integrators
        for (int i = 0; i < n_int; ++i)
            val /= s;
        // real zeros
        for (double wz : zeros)
            val *= (1.0 + s / wz);
        // real poles
        for (double wp : poles)
            val /= (1.0 + s / wp);
        return val;
    }
};

int main() {
    RealFactorBode sys;
    sys.K = 100.0;
    sys.n_int = 1;
    sys.n_diff = 0;
    sys.zeros = {10.0};
    sys.poles = {1.0, 100.0};

    std::vector<double> omega;
    for (int k = -2; k <= 3; ++k) {
        // 10 points per decade
        for (int j = 0; j < 10; ++j) {
            double exp10 = k + j / 10.0;
            omega.push_back(std::pow(10.0, exp10));
        }
    }

    for (double w : omega) {
        std::complex<double> s(0.0, w);
        std::complex<double> Gjw = sys.G(s);
        double mag = std::abs(Gjw);
        double mag_db = 20.0 * std::log10(mag);
        double phase = std::arg(Gjw) * 180.0 / M_PI;
        std::cout << w << " " << mag_db << " " << phase << "\n";
    }
    return 0;
}
