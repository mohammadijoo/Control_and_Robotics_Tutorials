/* Chapter12_Lesson4.cpp
   Nyquist and Nichols Plots (Introductory Level) — Frequency Response and Resonance

   Build (example):
     g++ -O2 -std=c++17 Chapter12_Lesson4.cpp -o Chapter12_Lesson4

   This program computes the complex frequency response G(jw) for a transfer function
   and writes a CSV suitable for plotting Nyquist (Re vs Im) and Nichols (phase vs dB).

   Output:
     - Chapter12_Lesson4_freqresp_cpp.csv
*/

#include <complex>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

static std::complex<double> poly_eval(const std::vector<double>& coeffs, const std::complex<double>& s) {
    // Horner evaluation: coeffs are highest power first
    std::complex<double> y(0.0, 0.0);
    for (double c : coeffs) {
        y = y * s + c;
    }
    return y;
}

static std::vector<double> logspace(double a, double b, int n) {
    // 10^a to 10^b
    std::vector<double> w;
    w.reserve(n);
    for (int k = 0; k < n; ++k) {
        double t = (n == 1) ? 0.0 : (double)k / (double)(n - 1);
        double p = a + (b - a) * t;
        w.push_back(std::pow(10.0, p));
    }
    return w;
}

int main() {
    // Example open-loop plant with a resonant mode and a first-order pole:
    // G(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * 1/(tau*s + 1)
    const double K = 5.0;
    const double wn = 10.0;
    const double zeta = 0.20;
    const double tau = 0.05;

    // Combined numerator and denominator polynomials
    // num(s) = K * wn^2
    std::vector<double> num = { K * wn * wn };

    // den(s) = (s^2 + a1*s + a0) * (b1*s + b0) expanded:
    // (s^2 + a1*s + a0)(b1*s + b0) = b1*s^3 + (b0 + a1*b1)*s^2 + (a1*b0 + a0*b1)*s + a0*b0
    const double a1 = 2.0 * zeta * wn;
    const double a0 = wn * wn;
    const double b1 = tau;
    const double b0 = 1.0;

    std::vector<double> den = {
        b1,
        (b0 + a1 * b1),
        (a1 * b0 + a0 * b1),
        (a0 * b0)
    };

    const int N = 1500;
    std::vector<double> w = logspace(-1.0, 2.5, N);

    std::ofstream ofs("Chapter12_Lesson4_freqresp_cpp.csv");
    if (!ofs) {
        std::cerr << "Cannot open output file.\n";
        return 1;
    }
    ofs << "omega_rad_s,ReG,ImG,Mag_dB,Phase_deg\n";

    for (double wk : w) {
        std::complex<double> s(0.0, wk); // j*w
        std::complex<double> Gjw = poly_eval(num, s) / poly_eval(den, s);

        double mag = std::abs(Gjw);
        double mag_db = 20.0 * std::log10(mag);
        double ph = std::arg(Gjw) * 180.0 / M_PI;

        ofs << wk << "," << Gjw.real() << "," << Gjw.imag() << "," << mag_db << "," << ph << "\n";
    }

    std::cout << "Wrote Chapter12_Lesson4_freqresp_cpp.csv\n";
    std::cout << "Plot Nyquist: ReG vs ImG. Plot Nichols: Phase_deg vs Mag_dB.\n";
    return 0;
}
