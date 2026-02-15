#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

// L(s) = k / (s + 1)^2
std::complex<double> loop_transfer(const std::complex<double> &s, double k) {
    std::complex<double> denom = (s + 1.0) * (s + 1.0);
    return k / denom;
}

int main() {
    double k = 5.0;
    std::vector<double> w;   // frequencies (rad/s)
    for (int i = -1; i <= 3; ++i) {
        // simple decade sweep: 10^i, 3 * 10^i, 10^(i+1)
        double w1 = std::pow(10.0, static_cast<double>(i));
        double w2 = 3.0 * w1;
        double w3 = 10.0 * w1;
        w.push_back(w1);
        w.push_back(w2);
        w.push_back(w3);
    }

    std::size_t N = w.size();
    for (std::size_t i = 0; i != N; ++i) {
        double wi = w[i];
        std::complex<double> s(0.0, wi);
        std::complex<double> L = loop_transfer(s, k);
        std::complex<double> S = 1.0 / (1.0 + L);
        double magS = std::abs(S);
        std::cout << "w = " << wi
                  << " rad/s, |S(jw)| = " << magS << std::endl;
    }

    return 0;
}
