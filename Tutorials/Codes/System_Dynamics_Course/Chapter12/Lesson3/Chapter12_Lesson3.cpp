// Chapter12_Lesson3.cpp
// System Dynamics — Chapter 12, Lesson 3
// Resonance, Bandwidth, and Quality Factor in Mechanical and Electrical Systems
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter12_Lesson3.cpp -o Chapter12_Lesson3
//
// Notes:
// - Uses only the C++ standard library (std::complex) for transfer-function evaluation.
// - For larger projects, pair this with Eigen (linear algebra) and/or Boost (numerics).

#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <limits>

struct SecondOrderParams {
    double wn;   // rad/s
    double zeta; // damping ratio
    double Q;    // quality factor
};

SecondOrderParams second_order_from_mck(double m, double c, double k) {
    if (m <= 0.0 || k <= 0.0) throw std::runtime_error("m and k must be positive.");
    double wn = std::sqrt(k / m);
    double zeta = c / (2.0 * std::sqrt(k * m));
    double Q = (zeta <= 0.0) ? std::numeric_limits<double>::infinity() : 1.0 / (2.0 * zeta);
    return {wn, zeta, Q};
}

bool resonance_frequency(double wn, double zeta, double &wr_out) {
    const double crit = 1.0 / std::sqrt(2.0);
    if (zeta >= crit) return false;
    wr_out = wn * std::sqrt(1.0 - 2.0 * zeta * zeta);
    return true;
}

// Half-power frequencies around the resonant peak of normalized low-pass second order
bool half_power_frequencies(double wn, double zeta, double &w1, double &w2) {
    double wr;
    if (!resonance_frequency(wn, zeta, wr)) return false;
    double inside = 1.0 - zeta * zeta;
    if (inside <= 0.0) return false;

    double r1_sq = 1.0 - 2.0 * zeta * zeta - 2.0 * zeta * std::sqrt(inside);
    double r2_sq = 1.0 - 2.0 * zeta * zeta + 2.0 * zeta * std::sqrt(inside);
    if (r1_sq <= 0.0 || r2_sq <= 0.0) return false;

    w1 = wn * std::sqrt(r1_sq);
    w2 = wn * std::sqrt(r2_sq);
    return true;
}

// Force->displacement transfer function: G(s) = 1 / (m s^2 + c s + k)
std::complex<double> G_force_to_disp(std::complex<double> s, double m, double c, double k) {
    return 1.0 / (m*s*s + c*s + k);
}

int main() {
    // Example mass-spring-damper
    double m = 1.0, c = 0.4, k = 100.0;
    auto p = second_order_from_mck(m, c, k);

    std::cout << "Mass–spring–damper parameters\n";
    std::cout << "  wn   = " << p.wn   << " rad/s\n";
    std::cout << "  zeta = " << p.zeta << "\n";
    std::cout << "  Q    = " << p.Q    << "\n";

    double wr;
    if (resonance_frequency(p.wn, p.zeta, wr)) {
        std::cout << "  wr   = " << wr << " rad/s\n";
        double w1, w2;
        if (half_power_frequencies(p.wn, p.zeta, w1, w2)) {
            double bw = w2 - w1;
            std::cout << "  w1   = " << w1 << " rad/s\n";
            std::cout << "  w2   = " << w2 << " rad/s\n";
            std::cout << "  BW   = " << bw << " rad/s\n";
            std::cout << "  Q_hp = " << (wr / bw) << " (wr/BW)\n";
            std::cout << "  BW approx (2 zeta wn) = " << (2.0*p.zeta*p.wn) << " rad/s\n";
        }
    } else {
        std::cout << "  wr   = (no resonant peak; zeta >= 1/sqrt(2))\n";
    }

    // Numeric sweep to confirm peak of |G(jw)| for force->displacement model
    std::vector<double> w;
    w.reserve(2000);
    for (int i = 0; i < 2000; ++i) {
        // logspace from 1e-1 to 1e3
        double a = -1.0 + 4.0 * (double)i / (2000.0 - 1.0);
        w.push_back(std::pow(10.0, a));
    }
    double w_peak = w[0];
    double mag_peak = 0.0;
    for (double wi : w) {
        std::complex<double> s(0.0, wi);
        double mag = std::abs(G_force_to_disp(s, m, c, k));
        if (mag > mag_peak) { mag_peak = mag; w_peak = wi; }
    }
    std::cout << "\nNumerical sweep (force->displacement) peak:\n";
    std::cout << "  w_peak ~ " << w_peak << " rad/s\n";
    std::cout << "  |G(jw_peak)| ~ " << mag_peak << "\n";

    // Series RLC quick computation
    double R = 10.0, L = 50e-3, C = 10e-6;
    double w0 = 1.0 / std::sqrt(L*C);
    double Q = (w0 * L) / R;
    double BW = R / L;
    std::cout << "\nSeries RLC:\n";
    std::cout << "  w0 = " << w0 << " rad/s\n";
    std::cout << "  Q  = " << Q  << "\n";
    std::cout << "  BW = " << BW << " rad/s\n";

    return 0;
}
