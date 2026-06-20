#include <iostream>
#include <complex>
#include <cmath>

// Plant P(s) = 1 / (s (s + 1)), controller C(s) = Kp + Kd s
// Evaluate L0(jw) on a log-spaced grid and estimate phase margin and delay margin.

std::complex<double> L0_of_jw(double w, double Kp, double Kd) {
    std::complex<double> j(0.0, 1.0);
    std::complex<double> s = j * w;
    std::complex<double> P = 1.0 / (s * (s + 1.0));
    std::complex<double> C = Kp + Kd * s;
    return C * P;
}

int main() {
    double Kp = 40.0;
    double Kd = 2.0;

    double w_min = 0.1;
    double w_max = 100.0;
    int N = 200;

    double wgc = 0.0;      // estimated gain crossover frequency
    double phi_deg = 0.0;  // phase at crossover

    // Scan over logarithmic grid to find |L0(jw)| ~ 1
    double prev_mag = 0.0;
    double prev_w = w_min;
    for (int k = 0; k <= N; ++k) {
        double alpha = static_cast<double>(k) / N;
        double w = w_min * std::pow(w_max / w_min, alpha);
        std::complex<double> L = L0_of_jw(w, Kp, Kd);
        double mag = std::abs(L);

        if (k > 0) {
            // Look for a sign change in (mag - 1), indicating a crossing of unity gain
            double f_prev = prev_mag - 1.0;
            double f_curr = mag - 1.0;
            if (f_prev * f_curr <= 0.0) {
                // Linear interpolation for a better estimate of wgc
                double t = f_prev / (f_prev - f_curr + 1e-12);
                wgc = prev_w + t * (w - prev_w);
                break;
            }
        }
        prev_mag = mag;
        prev_w = w;
    }

    if (wgc > 0.0) {
        std::complex<double> Lc = L0_of_jw(wgc, Kp, Kd);
        phi_deg = std::arg(Lc) * 180.0 / M_PI;
        double pm_deg = 180.0 + phi_deg;     // phase margin in degrees
        double pm_rad = pm_deg * M_PI / 180.0;
        double D_max = pm_rad / wgc;         // delay margin in seconds

        std::cout << "Gain crossover wgc = " << wgc << " rad/s\n";
        std::cout << "Phase at wgc = " << phi_deg << " deg\n";
        std::cout << "Phase margin = " << pm_deg << " deg\n";
        std::cout << "Approximate delay margin D_max = "
                  << D_max << " s\n";
    } else {
        std::cout << "No gain crossover found in the scanned range.\n";
    }

    return 0;
}
