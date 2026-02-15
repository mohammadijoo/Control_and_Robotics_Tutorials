#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

using cd = std::complex<double>;

cd P0(cd s, double J, double b, double Km) {
    return Km / (J * s * s + b * s);
}

cd Gf(cd s, double w_f, double zeta_f) {
    return (w_f * w_f) / (s * s + 2.0 * zeta_f * w_f * s + w_f * w_f);
}

int main() {
    double J = 0.01;
    double b = 0.1;
    double Km = 1.0;
    double w_f = 200.0;
    double zeta_f = 0.05;

    // Logarithmic frequency grid
    std::vector<double> w;
    for (int k = 0; k <= 400; ++k) {
        double exponent = std::log10(1.0) + (4.0 * k) / 400.0;
        w.push_back(std::pow(10.0, exponent));
    }

    double maxDelta = 0.0;
    for (double wi : w) {
        cd s(0.0, wi);
        cd P0jw = P0(s, J, b, Km);
        cd Gfjw = Gf(s, w_f, zeta_f);
        cd Delta_m = Gfjw - cd(1.0, 0.0);
        double mag = std::abs(Delta_m);
        if (mag > maxDelta) {
            maxDelta = mag;
        }
    }

    std::cout << "Maximum |Delta_m(jw)| over the grid = " << maxDelta << std::endl;

    // In a robot controller, this estimate can guide bandwidth selection:
    // ensure loop gain is low around frequencies where |Delta_m(jw)| is large.
    return 0;
}
