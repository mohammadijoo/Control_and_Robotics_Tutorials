#include <iostream>
#include <complex>
#include <vector>

using cd = std::complex<double>;

cd L_of_jw(double w, double K, double p, double Ldelay) {
    cd s(0.0, w);                 // s = j w
    cd G0 = K / (s - cd(p, 0.0)); // K / (s - p), p > 0 (RHP pole)
    cd delay = std::exp(-Ldelay * s); // exp(-L s)
    return G0 * delay;
}

int main() {
    double K = 2.0;
    double p = 1.0;
    double Ldelay = 0.05; // 50 ms

    std::vector<double> w_grid;
    for (int k = 0; k <= 500; ++k) {
        w_grid.push_back(0.1 * k); // 0 ... 50 rad/s
    }

    for (double w : w_grid) {
        cd L = L_of_jw(w, K, p, Ldelay);
        std::cout << w << " "
                  << std::real(L) << " "
                  << std::imag(L) << std::endl;
    }

    return 0;
}
