#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

using Vec = std::vector<double>;
using cd  = std::complex<double>;

cd evalPoly(const Vec& coeffs, cd s) {
    cd result(0.0, 0.0);
    for (size_t i = 0; i < coeffs.size(); ++i) {
        size_t power = coeffs.size() - 1 - i;
        result += coeffs[i] * std::pow(s, static_cast<int>(power));
    }
    return result;
}

double magResponse(const Vec& num, const Vec& den, double omega) {
    cd s(0.0, omega); // s = j omega
    cd H = evalPoly(num, s) / evalPoly(den, s);
    return std::abs(H);
}

int main() {
    // Example: notch filter H(s) = (s^2 + w0^2) / (s^2 + (w0/Q) s + w0^2)
    double w0 = 50.0;
    double Q  = 10.0;

    Vec num{1.0, 0.0, w0 * w0};
    Vec den{1.0, w0 / Q, w0 * w0};

    for (double w = 10.0; w <= 1000.0; w *= 1.5) {
        double mag = magResponse(num, den, w);
        std::cout << "omega = " << w
                  << ", |H(j omega)| = " << mag << std::endl;
    }
    return 0;
}
