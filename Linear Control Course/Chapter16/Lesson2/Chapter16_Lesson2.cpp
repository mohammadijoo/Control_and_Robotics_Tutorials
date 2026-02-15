#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

// Example plant: G(s) = K / (s^2 + 2*zeta*wn*s + wn^2)
std::complex<double> G_of_jw(double K, double zeta, double wn, double w)
{
    std::complex<double> jw(0.0, w);
    std::complex<double> denom = jw * jw + 2.0 * zeta * wn * jw
                                  + std::complex<double>(wn * wn, 0.0);
    return std::complex<double>(K, 0.0) / denom;
}

int main()
{
    double K   = 10.0;
    double zeta = 0.3;
    double wn   = 5.0;

    std::vector<double> w_vec;
    for (int i = 0; i <= 400; ++i) {
        double w = std::pow(10.0, -1.0 + 3.0 * i / 400.0); // 10^-1 .. 10^2
        w_vec.push_back(w);
    }

    double Mr = 0.0;
    double w_mr = 0.0;

    for (double w : w_vec) {
        std::complex<double> L = G_of_jw(K, zeta, wn, w);
        std::complex<double> T = L / (1.0 + L);

        double M   = std::abs(T);
        double psi = std::arg(T);         // radians
        double Lm  = std::abs(L);
        double phi = std::arg(L);
        double L_db = 20.0 * std::log10(Lm);

        // Here (phi, L_db) is the Nichols point, (M, psi) is closed-loop response.
        if (M > Mr) {
            Mr   = M;
            w_mr = w;
        }

        // In a ROS node, you might publish these quantities for analysis.
    }

    double Mr_db = 20.0 * std::log10(Mr);
    std::cout << "Closed-loop peak Mr = " << Mr
              << " (" << Mr_db << " dB)"
              << " at w = " << w_mr << " rad/s\n";

    return 0;
}
