#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

using Complex = std::complex<double>;

Complex L(Complex s, double K) {
    // Example L(s) = K / ((s + 0.1)(s + 1))
    return K / ((s + Complex(0.1, 0.0)) * (s + Complex(1.0, 0.0)));
}

int main() {
    const double K = 5.0;
    const int N = 4000;
    const double w_min = 1e-2;
    const double w_max = 1e2;

    std::vector<Complex> F_values;
    F_values.reserve(N);

    // Log-spaced frequency grid
    for (int k = 0; k < N; ++k) {
        double alpha = static_cast<double>(k) / (N - 1);
        double w = std::pow(10.0, std::log10(w_min) +
                                   alpha * (std::log10(w_max) - std::log10(w_min)));
        Complex s(0.0, w);
        Complex Ljw = L(s, K);
        F_values.push_back(Complex(1.0, 0.0) + Ljw); // F(s) = 1 + L(s)
    }

    // Approximate total change in argument of F(s) to estimate encirclements
    double total_phase_change = 0.0;
    for (int k = 1; k < N; ++k) {
        double phi_prev = std::atan2(F_values[k-1].imag(), F_values[k-1].real());
        double phi_curr = std::atan2(F_values[k].imag(), F_values[k].real());
        double dphi = phi_curr - phi_prev;

        // Wrap into [-pi, pi]
        if (dphi > M_PI) dphi -= 2.0 * M_PI;
        if (dphi < -M_PI) dphi += 2.0 * M_PI;

        total_phase_change += dphi;
    }

    double N_ccw = total_phase_change / (2.0 * M_PI);
    double N_clockwise = -N_ccw;

    std::cout << "Approximate clockwise encirclements of -1: N = "
              << N_clockwise << std::endl;

    // In a robotics setting, this tool can be integrated into an offline tuning workflow
    // for joint controllers or mobile robot steering controllers.
    return 0;
}
