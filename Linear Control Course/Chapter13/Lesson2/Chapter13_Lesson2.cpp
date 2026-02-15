#include <iostream>
#include <complex>
#include <vector>

int main() {
    double K = 2.0;
    double T = 0.1;
    std::vector<double> omega = {0.1, 1.0, 10.0, 100.0}; // rad/s

    std::complex<double> j(0.0, 1.0);

    auto G_of_s = [&](std::complex<double> s) {
        return K / (1.0 + T * s);
    };

    for (double w : omega) {
        std::complex<double> s = j * w;
        std::complex<double> G = G_of_s(s);

        double mag = std::abs(G);
        double phase = std::arg(G); // radians

        std::cout << "w = " << w
                  << " rad/s, |G(jw)| = " << mag
                  << ", phase = " << phase
                  << " rad" << std::endl;
    }

    return 0;
}
