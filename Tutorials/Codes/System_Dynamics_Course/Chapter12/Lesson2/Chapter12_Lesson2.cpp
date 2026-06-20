/*
Chapter 12 - Lesson 2: Bode Plots: Magnitude, Phase, Asymptotes, and Construction Rules
System Dynamics (Control Engineering)

This C++ program computes frequency response G(jw) for a rational transfer function
given numerator/denominator polynomials and writes CSV output suitable for plotting.

Compile (example):
  g++ -O2 -std=c++17 Chapter12_Lesson2.cpp -o bode

Run:
  ./bode

Output:
  bode_output.csv with columns: w, mag_db, phase_deg_unwrapped
*/

#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <fstream>
#include <iomanip>

static std::complex<double> polyval(const std::vector<double>& c,
                                    const std::complex<double>& s) {
    std::complex<double> y(0.0, 0.0);
    for (double a : c) {
        y = y * s + a;
    }
    return y;
}

static double mag_db(const std::complex<double>& z) {
    return 20.0 * std::log10(std::abs(z));
}

static double phase_deg(const std::complex<double>& z) {
    return std::atan2(z.imag(), z.real()) * 180.0 / M_PI;
}

static void unwrap_phase(std::vector<double>& ph_deg) {
    for (size_t k = 1; k < ph_deg.size(); ++k) {
        double d = ph_deg[k] - ph_deg[k - 1];
        if (d > 180.0) {
            for (size_t j = k; j < ph_deg.size(); ++j) ph_deg[j] -= 360.0;
        } else if (d < -180.0) {
            for (size_t j = k; j < ph_deg.size(); ++j) ph_deg[j] += 360.0;
        }
    }
}

int main() {
    // Example transfer function:
    //   G(s) = 10 * (1 + s/1) / ( s * (1 + s/10) )
    // num: 10*(s + 1) -> [10, 10]
    // den: s*(0.1 s + 1) -> 0.1 s^2 + s + 0 -> [0.1, 1, 0]
    const std::vector<double> num = {10.0, 10.0};
    const std::vector<double> den = {0.1, 1.0, 0.0};

    const int N = 2000;
    const double w_min = 1e-2;
    const double w_max = 1e3;

    std::vector<double> w(N), mag(N), ph(N);

    for (int k = 0; k < N; ++k) {
        double t = static_cast<double>(k) / static_cast<double>(N - 1);
        // logspace: w = w_min * (w_max/w_min)^t
        double wk = w_min * std::pow(w_max / w_min, t);
        w[k] = wk;

        std::complex<double> s(0.0, wk);  // jw
        std::complex<double> Gjw = polyval(num, s) / polyval(den, s);

        mag[k] = mag_db(Gjw);
        ph[k]  = phase_deg(Gjw);
    }

    unwrap_phase(ph);

    std::ofstream f("bode_output.csv");
    f << "w,mag_db,phase_deg\n";
    f << std::setprecision(12);
    for (int k = 0; k < N; ++k) {
        f << w[k] << "," << mag[k] << "," << ph[k] << "\n";
    }
    f.close();

    std::cout << "Wrote bode_output.csv (" << N << " points)." << std::endl;
    std::cout << "Plot using your preferred tool (e.g., Python/matplotlib, gnuplot)." << std::endl;
    return 0;
}
