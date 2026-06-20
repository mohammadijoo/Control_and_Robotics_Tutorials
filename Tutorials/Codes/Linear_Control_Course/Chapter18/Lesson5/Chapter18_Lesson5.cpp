#include <iostream>
#include <complex>
#include <vector>
#include <cmath>
// In robotics projects, Eigen is frequently used:
// #include <Eigen/Dense>

using std::complex;
using std::vector;

// Plant P(s) = 1 / (s (s + 1) (s + 2))
complex<double> P_eval(double w) {
    complex<double> jw(0.0, w);
    return 1.0 / (jw * (jw + 1.0) * (jw + 2.0));
}

// PI controller C(s) = (Kp s + Ki) / s
complex<double> C_eval(double w, double Kp, double Ki) {
    complex<double> jw(0.0, w);
    return (Kp * jw + Ki) / jw;
}

int main() {
    double Kp1 = 2.0, Ki1 = 1.0;
    double Kp2 = 6.0, Ki2 = 4.0;

    vector<double> w;
    int N = 200;
    double wmin = 0.01, wmax = 100.0;
    for (int i = 0; i < N; ++i) {
        double logw = std::log10(wmin) +
                      (std::log10(wmax) - std::log10(wmin)) * i / (N - 1);
        w.push_back(std::pow(10.0, logw));
    }

    std::cout << "# w  |L1(jw)|  |L2(jw)|" << std::endl;
    for (double wi : w) {
        complex<double> P = P_eval(wi);
        complex<double> C1 = C_eval(wi, Kp1, Ki1);
        complex<double> C2 = C_eval(wi, Kp2, Ki2);

        complex<double> L1 = C1 * P;
        complex<double> L2 = C2 * P;

        double magL1 = std::abs(L1);
        double magL2 = std::abs(L2);

        std::cout << wi << "  "
                  << magL1 << "  "
                  << magL2 << std::endl;
    }

    // In a full implementation, you would also numerically simulate the
    // step response by discretizing the state-space realization, or by
    // using an ODE integrator (e.g. Runge-Kutta) on the differential equations.

    return 0;
}
