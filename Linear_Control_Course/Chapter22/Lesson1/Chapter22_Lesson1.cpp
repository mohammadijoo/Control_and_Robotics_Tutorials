#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

int main() {
    using std::complex;
    using std::cout;
    using std::endl;

    double K = 10.0;               // proportional gain
    complex<double> j(0.0, 1.0);   // imaginary unit

    // logarithmic frequency grid
    std::vector<double> w;
    for (int k = 0; k <= 40; ++k) {
        double wk = std::pow(10.0, -1.0 + 0.1 * k); // 10^(-1) ... 10^3
        w.push_back(wk);
    }

    for (double wk : w) {
        complex<double> s = j * wk;
        complex<double> P = 1.0 / (s + 1.0);   // P(s) = 1 / (s + 1)
        complex<double> C = K;                 // C(s) = K
        complex<double> L = C * P;             // loop transfer
        complex<double> S = 1.0 / (1.0 + L);   // sensitivity

        double magS = std::abs(S);
        double magS_dB = 20.0 * std::log10(magS);

        cout << "w = " << wk
             << ", |S(jw)| = " << magS
             << " (" << magS_dB << " dB)" << endl;
    }

    return 0;
}
