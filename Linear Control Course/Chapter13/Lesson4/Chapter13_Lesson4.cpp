#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

int main() {
    double zeta = 0.6;
    double wn   = 13.5; // rad/s

    // Time discretization for step response simulation (Euler integration)
    double dt = 0.0005;
    double T_end = 2.0;
    int N = static_cast<int>(T_end / dt);

    std::vector<double> t(N), y(N);
    double y_val = 0.0;     // output
    double y_dot = 0.0;     // first derivative
    double u = 1.0;         // unit step

    for (int k = 0; k < N; ++k) {
        t[k] = k * dt;
        // Second-order ODE: y'' + 2*zeta*wn*y' + wn^2*y = wn^2*u
        double y_ddot = wn*wn * (u - y_val) - 2.0 * zeta * wn * y_dot;
        y_dot += dt * y_ddot;
        y_val += dt * y_dot;
        y[k] = y_val;
    }

    // Estimate overshoot and settling time (2% band)
    double y_final = y.back();
    double peak = y[0];
    for (double v : y) {
        if (v > peak) peak = v;
    }
    double Mp = (peak - y_final) / y_final;

    int last_outside = 0;
    for (int k = 0; k < N; ++k) {
        if (std::fabs(y[k] - y_final) > 0.02 * std::fabs(y_final)) {
            last_outside = k;
        }
    }
    double ts = t[last_outside];

    std::cout << "Mp = " << Mp * 100.0 << "%\n";
    std::cout << "ts = " << ts << " s\n";

    // Frequency response |T(jw)| for a range of omega
    std::vector<double> omega;
    for (double w = 0.1; w <= 100.0; w *= 1.05) {
        omega.push_back(w);
    }

    double Mr = 0.0;
    double wr = 0.0;
    double target = 1.0 / std::sqrt(2.0);
    double wB = NAN;
    bool found_bw = false;

    for (double w : omega) {
        std::complex<double> s(0.0, w);
        std::complex<double> num(wn*wn, 0.0);
        std::complex<double> den = s*s + 2.0*zeta*wn*s + wn*wn;
        std::complex<double> Tjw = num / den;
        double mag = std::abs(Tjw);

        if (mag > Mr) {
            Mr = mag;
            wr = w;
        }
        if (!found_bw && mag <= target) {
            wB = w;
            found_bw = true;
        }
    }

    std::cout << "Mr = " << Mr << ", wr = " << wr << " rad/s\n";
    std::cout << "wB (approx) = " << wB << " rad/s\n";

    return 0;
}
