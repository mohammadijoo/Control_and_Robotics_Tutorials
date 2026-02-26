#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

int main() {
    using std::complex;
    using std::vector;

    double J = 0.01;
    double B = 0.1;
    double K = 1.0;
    double Kp = 5.0;

    auto L_of_s = [&] (complex<double> s) {
        return Kp * K / (J * s * s + B * s);
    };

    // Log-spaced frequency grid
    int N = 400;
    double w_min = 1e-2;
    double w_max = 1e3;

    vector<complex<double> nyquist_points;
    nyquist_points.reserve(2 * N);

    // Positive frequencies
    for (int k = 0; k < N; ++k) {
        double alpha = static_cast<double>(k) / (N - 1);
        double w = w_min * std::pow(w_max / w_min, alpha);
        complex<double> s(0.0, w);
        nyquist_points.push_back(L_of_s(s));
    }

    // Negative frequencies (conjugate symmetry)
    for (int k = N - 1; k >= 0; --k) {
        nyquist_points.push_back(std::conj(nyquist_points[k]));
    }

    // At this point, nyquist_points can be exported to a file and plotted
    // with gnuplot or a visualization tool integrated into a robotics tuning GUI.

    for (const auto &z : nyquist_points) {
        std::cout << z.real() << " " << z.imag() << "\n";
    }

    return 0;
}
