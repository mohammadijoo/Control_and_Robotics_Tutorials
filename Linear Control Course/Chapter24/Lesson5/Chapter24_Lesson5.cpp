#include <iostream>
#include <complex>
#include <vector>
#include <Eigen/Dense>

// Simple second-order loop: L(s) = Kc * K / (J s^2 + B s)
// approximated at s = j w.
double sensitivity_peak(double J, double Kc, double K, double B)
{
    using std::complex;
    const double w_min = 0.1;
    const double w_max = 100.0;
    const int N = 2000;

    double Ms = 0.0;
    for (int k = 0; k < N; ++k) {
        double w = w_min * std::exp(std::log(w_max / w_min) * k / (N - 1));
        complex<double> s(0.0, w);
        complex<double> G = K / (J * s * s + B * s);
        complex<double> L = Kc * G;
        complex<double> S = 1.0 / (1.0 + L);
        double mag = std::abs(S);
        if (mag > Ms) Ms = mag;
    }
    return Ms;
}

int main()
{
    double J_nom = 0.01;
    double B_nom = 0.1;
    double K_gain = 1.0;
    double Kc = 2.0;

    std::vector<double> J_values = {0.005, 0.01, 0.015};

    for (double J : J_values) {
        double Ms = sensitivity_peak(J, Kc, K_gain, B_nom);
        std::cout << "J = " << J
                  << ", approximated M_S = " << Ms << std::endl;
    }

    return 0;
}
